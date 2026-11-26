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
#include "leg_stow_gate.h"
#include "platform_relay.h"
#include "version_check.h"
#include "fake_hal.h"

// ── Driver-local leaf/handler stubs (the disjoint set fake_hal does NOT define) ─
//  fake_hal provides jugglebot_commands_allowed / can_jugglebot_send / now_wall_us /
//  micros64. These are the remainder rpc.cpp links against. Everything here records
//  or returns a settable value so a TEST_CASE can assert routing + gate decisions.
namespace CanBridge {

static bool     g_bus_transmittable = true;   // drives jugglebot_bus_transmittable()
static int      g_bb_sent = 0;                // can_bb_tx call count
static ODrive::CanFrame g_bb_last{};          // recording CAN1 (BB) TX
static bool     g_bb_send_ok = true;
static uint8_t  g_bb_last_cls = 0xFF;         // the TxCls rpc.cpp charged the BB relay to
static TxResult g_bb_result = TxResult::MAILBOX;   // outcome the fake returns when "ok"
static int      g_clear_calls = 0;            // fault_notify_clear_errors count
static int      g_reboot_calls = 0;           // fault_notify_reboot_started count
static bool     g_stow_pending = false;       // drives fault_stow_pending()
static uint8_t  g_homing_axis = 0xFF, g_activate_axis = 0xFF, g_deactivate_axis = 0xFF;
static uint16_t g_homing_ret = JbUdp::RpcStatus::OK;
static uint16_t g_activate_ret = JbUdp::RpcStatus::OK;
static uint16_t g_deactivate_ret = JbUdp::RpcStatus::OK;
static bool     g_mpc_active = false;         // drives fault_mpc_active()
static uint16_t g_version_len = 8;            // version_fill_blob returned length
// FW 22 GET_HAND_TORQUE_SCALE's firmware gate reads the hand's cached Get_Version.
static bool    g_hand_ver_have = false;
static uint8_t g_hand_ver_raw[8] = {0};
static int      g_tilt_calls = 0, g_state_read_calls = 0, g_state_write_calls = 0;
static int      g_hand_fan_calls = 0;
static JbUdp::RpcArgs::ArgHandFanSet g_hand_fan_args{};
static int      g_imu_ctrl_calls = 0, g_imu_read_calls = 0;   // FW 45 Relay::platform_imu_* stubs
static JbUdp::RpcArgs::ArgPlatformImuCtrl g_imu_ctrl_args{};
static JbUdp::RpcArgs::ArgPlatformImuRead g_imu_read_args{};
static bool     g_imu_read_armed = false;     // the armed flag rpc.cpp sampled + passed in
static int      g_bb_fw_calls = 0;            // Relay::bb_fw_* stub call count
static uint16_t g_bb_fw_method = 0;           // which bb_fw_* stub ran (the RpcMethod it serves)
static Relay::BbFwGate g_bb_fw_gate{};        // the gate rpc.cpp sampled + passed in

static void drv_reset() {
  g_bus_transmittable = true;
  g_bb_sent = 0; g_bb_last = ODrive::CanFrame{}; g_bb_send_ok = true;
  g_bb_last_cls = 0xFF; g_bb_result = TxResult::MAILBOX;
  g_clear_calls = 0; g_reboot_calls = 0; g_stow_pending = false;
  g_homing_axis = g_activate_axis = g_deactivate_axis = 0xFF;
  g_homing_ret = g_activate_ret = g_deactivate_ret = JbUdp::RpcStatus::OK;
  g_mpc_active = false;
  g_version_len = 8;
  g_tilt_calls = g_state_read_calls = g_state_write_calls = 0;
  g_hand_fan_calls = 0;
  g_imu_ctrl_calls = g_imu_read_calls = 0; g_imu_read_armed = false;
  g_imu_ctrl_args = JbUdp::RpcArgs::ArgPlatformImuCtrl{}; g_imu_read_args = JbUdp::RpcArgs::ArgPlatformImuRead{};
  g_hand_ver_have = false; memset(g_hand_ver_raw, 0, 8);
  g_bb_fw_calls = 0; g_bb_fw_method = 0; g_bb_fw_gate = Relay::BbFwGate{};
}

// ── can_buses.h ──
bool jugglebot_bus_transmittable() { return g_bus_transmittable; }
TxResult can_bb_tx(const ODrive::CanFrame& f, uint8_t cls) {
  g_bb_sent++; g_bb_last = f; g_bb_last_cls = cls;
  return g_bb_send_ok ? g_bb_result : TxResult::FAILED;
}
// Cone-role bus (WAVE_CMD, FW 33). g_cone_result models can_cone_tx's own
// presence gate: FAILED = no cone-bus partner heard, nothing queued.
static int      g_cone_sent = 0;
static ODrive::CanFrame g_cone_last{};
static uint8_t  g_cone_last_cls = 0xFF;
static TxResult g_cone_result = TxResult::MAILBOX;
TxResult can_cone_tx(const ODrive::CanFrame& f, uint8_t cls) {
  if (g_cone_result != TxResult::FAILED) { g_cone_sent++; g_cone_last = f; g_cone_last_cls = cls; }
  return g_cone_result;
}

// ── fault_machine.h ──
void fault_notify_clear_errors()  { g_clear_calls++; }
static int      g_axis_clear_calls = 0;       // FW 52: fault_notify_clear_axis_errors count
static uint8_t  g_axis_clear_axis = 0xFF;
void fault_notify_clear_axis_errors(uint8_t axis) { g_axis_clear_calls++; g_axis_clear_axis = axis; }
void fault_notify_reboot_started() { g_reboot_calls++; }
bool fault_stow_pending()          { return g_stow_pending; }
bool fault_can_bus_down()          { return false; }   // link only: leg_stow_gate.o's stow_descent_step reads it (never called here)
bool fault_mpc_active()            { return g_mpc_active; }

// ── leg_homing/activate/deactivate.h (entry points only — record + settable ret) ──
uint16_t homing_request(uint8_t axis)     { g_homing_axis = axis; return g_homing_ret; }
uint16_t activate_request(uint8_t axis)   { g_activate_axis = axis; return g_activate_ret; }
static float    g_activate_to_targets[6] = {0};   // ACTIVATE_TO (FW 51): the args as received
static int      g_activate_to_calls = 0;
static uint16_t g_activate_to_ret = JbUdp::RpcStatus::OK;
uint16_t activate_to_request(const float t[6]) {
  for (int i = 0; i < 6; ++i) g_activate_to_targets[i] = t[i];
  g_activate_to_calls++;
  return g_activate_to_ret;
}
uint16_t deactivate_request(uint8_t axis) { g_deactivate_axis = axis; return g_deactivate_ret; }
// HAND_MOVE_TO (FW 26): record the decoded args + the req_id dispatch handed over,
// and model the reply mailbox the activate task posts into (one slot is enough here).
static uint8_t  g_hm_axis = 0xEE;
static float    g_hm_target = -1.0f, g_hm_vel = -1.0f;
static uint16_t g_hm_req_id = 0, g_hm_ret = JbUdp::RpcStatus::OK;
static bool          g_hm_reply_ready = false;
static HandMoveReply g_hm_reply;
uint16_t hand_move_request(uint8_t axis, float target_rev, float vel_rps, uint16_t req_id) {
  g_hm_axis = axis; g_hm_target = target_rev; g_hm_vel = vel_rps; g_hm_req_id = req_id;
  return g_hm_ret;
}
bool hand_move_pop_reply(HandMoveReply& out) {
  if (!g_hm_reply_ready) return false;
  out = g_hm_reply; g_hm_reply_ready = false; return true;
}
uint32_t hand_move_reply_drops() { return 0; }

// ── platform_relay.h ──
namespace Relay {
uint16_t tilt_read()  { g_tilt_calls++; return JbUdp::RpcStatus::OK; }
uint16_t state_read() { g_state_read_calls++; return JbUdp::RpcStatus::OK; }
uint16_t state_write(const JbUdp::RpcArgs::ArgRobotState&) { g_state_write_calls++; return JbUdp::RpcStatus::OK; }
uint16_t hand_fan_set(const JbUdp::RpcArgs::ArgHandFanSet& a) { g_hand_fan_calls++; g_hand_fan_args = a; return JbUdp::RpcStatus::OK; }
// Platform IMU (FW 45). Recording stubs: the frame layout, the op whitelist and the
// Armed gap clamp live in platform_relay.cpp (test_platform_relay.cpp).
uint16_t platform_imu_ctrl(const JbUdp::RpcArgs::ArgPlatformImuCtrl& a) { g_imu_ctrl_calls++; g_imu_ctrl_args = a; return JbUdp::RpcStatus::OK; }
uint16_t platform_imu_read(const JbUdp::RpcArgs::ArgPlatformImuRead& a, bool armed) { g_imu_read_calls++; g_imu_read_args = a; g_imu_read_armed = armed; return JbUdp::RpcStatus::OK; }
uint8_t imu_read_effective_gap(uint8_t g, bool) { return g; }
// Platform firmware-over-CAN (FW 19). Link stubs only — the real gating
// (mpc_active ⇒ ERR_REJECTED, n outside 1..5 ⇒ ERR_BAD_ARGS, the CAN3 gate) lives
// in platform_relay.cpp and is exercised through test_platform_relay.cpp.
uint16_t platform_fw_begin(const JbUdp::RpcArgs::ArgPlatformFwBegin&, bool) { return JbUdp::RpcStatus::OK; }
uint16_t platform_fw_data(const JbUdp::RpcArgs::ArgPlatformFwData&, bool) { return JbUdp::RpcStatus::OK; }
uint16_t platform_fw_verify(const JbUdp::RpcArgs::ArgPlatformFwVerify&, bool) { return JbUdp::RpcStatus::OK; }
uint16_t platform_fw_commit(bool) { return JbUdp::RpcStatus::OK; }
// Ball Butler firmware-over-CAN (FW 21). Recording stubs: the frame layouts and
// the gate DECISIONS are exercised in test_platform_relay.cpp; here the question
// is only whether rpc.cpp routes each method and samples the gate from bb_state.
static uint16_t bb_fw_stub(uint16_t method, const BbFwGate& g) {
  g_bb_fw_calls++; g_bb_fw_method = method; g_bb_fw_gate = g;
  return JbUdp::RpcStatus::OK;
}
uint16_t bb_fw_begin(const JbUdp::RpcArgs::ArgPlatformFwBegin&, const BbFwGate& g) { return bb_fw_stub(JbUdp::RpcMethod::BB_FW_BEGIN, g); }
uint16_t bb_fw_data(const JbUdp::RpcArgs::ArgPlatformFwData&, const BbFwGate& g) { return bb_fw_stub(JbUdp::RpcMethod::BB_FW_DATA, g); }
uint16_t bb_fw_verify(const JbUdp::RpcArgs::ArgPlatformFwVerify&, const BbFwGate& g) { return bb_fw_stub(JbUdp::RpcMethod::BB_FW_VERIFY, g); }
uint16_t bb_fw_commit(const BbFwGate& g) { return bb_fw_stub(JbUdp::RpcMethod::BB_FW_COMMIT, g); }
uint16_t bb_fw_info(const BbFwGate& g) { return bb_fw_stub(JbUdp::RpcMethod::BB_FW_INFO, g); }
}  // namespace Relay

// ── version_check.h ──
uint16_t version_fill_blob(uint8_t* out, uint16_t cap) {
  const uint16_t n = g_version_len < cap ? g_version_len : cap;
  for (uint16_t i = 0; i < n; ++i) out[i] = 0;
  return n;
}
// The Ball Butler twin (FW 20). Shares g_version_len so a case can drive both
// GET_AXIS_VERSIONS and GET_BB_AXIS_VERSIONS down the zero-length branch that
// makes dispatch answer ERR_BAD_ARGS.
bool version_raw_copy(uint8_t axis, uint8_t* out8) {
  if (axis != HAND_AXIS || !g_hand_ver_have) return false;
  memcpy(out8, g_hand_ver_raw, 8);
  return true;
}
uint16_t bb_version_fill_blob(uint8_t* out, uint16_t cap) {
  const uint16_t n = g_version_len < cap ? g_version_len : cap;
  for (uint16_t i = 0; i < n; ++i) out[i] = 0;
  return n;
}

// ── udp_link.h (link deps pulled in by rpc_server_init/on_request; never called) ──
// Records the last RPC frame sent (on_request's immediate replies and
// rpc_service's deferred ones both land here).
static int      g_udp_sends = 0;
static uint8_t  g_udp_last_type = 0;
static uint8_t  g_udp_last[64];
static uint16_t g_udp_last_len = 0;
bool udp_send_rpc(uint8_t t, const uint8_t* p, uint16_t n) {
  ++g_udp_sends; g_udp_last_type = t;
  g_udp_last_len = (n <= sizeof(g_udp_last)) ? n : (uint16_t)sizeof(g_udp_last);
  memcpy(g_udp_last, p, g_udp_last_len);
  return true;
}
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

static void reset_all() {
  fake_reset(); drv_reset(); Rpc::hand_torque_scale_reset();
  CanBridge::leg_stow_gate_init();   // FW 53: the gate's CL stamps, HELD mask, census, bench witnesses
  g_axis_clear_calls = 0; g_axis_clear_axis = 0xFF;
  g_cone_sent = 0; g_cone_last = ODrive::CanFrame{}; g_cone_last_cls = 0xFF;
  g_cone_result = TxResult::MAILBOX;
}

using namespace JbUdp;
using JbUdp::RpcArgs::ArgAxisState;
using JbUdp::RpcArgs::ArgPlatformFwBegin;
using JbUdp::RpcArgs::ArgPlatformFwData;
using JbUdp::RpcArgs::ArgPlatformFwVerify;
using JbUdp::RpcArgs::ArgAxisOnly;
using JbUdp::RpcArgs::ArgAbsPosition;
using JbUdp::RpcArgs::ArgSdoRead;
using JbUdp::RpcArgs::ArgBbThrow;
using JbUdp::RpcArgs::ArgWaveCmd;
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
  ArgSdoRead sd{}; sd.axis = HAND_AXIS; sd.endpoint = 726;
  CHECK(call(RpcMethod::SDO_READ, sd) == RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);
}

// ── the Phase-6 gate-basis split (the 2026-06-27 just-repowered-bus crux) ──────

TEST_CASE("stale bus (commands_allowed=false) but transmittable=true: CLEAR reaches it, SET_AXIS_STATE does not") {
  reset_all();
  fake_set_commands_allowed(false);   // heartbeat-stale
  g_bus_transmittable = true;         // but electrically alive (SYNCH=1)
  // CLEAR_ERRORS gates on bus-transmittable → allowed. FW 52 (BUG-99 D2): a
  // PER-AXIS clear clears that axis only; it no longer calls the global release
  // (FW 49-51 pinned g_clear_calls == 1 here).
  ArgAxisOnly c{}; c.axis = 0;
  CHECK(call(RpcMethod::CLEAR_ERRORS, c) == RpcStatus::OK);
  CHECK(g_clear_calls == 0);
  CHECK(g_axis_clear_calls == 1);
  CHECK(g_axis_clear_axis == 0);
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

TEST_CASE("item 20: REBOOT_ODRIVES is rejected during a deferred stow (no TX, no latch)") {
  // A pending/in-progress stow must block REBOOT — a reboot mid-descent disarms the
  // raised legs (gravity drop). Mirrors the cold-start requests' stow-pending reject.
  reset_all();
  g_stow_pending = true;

  ArgAxisOnly leg{}; leg.axis = 0;
  CHECK(call(RpcMethod::REBOOT_ODRIVES, leg) == RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);       // nothing rebooted
  CHECK(g_reboot_calls == 0);          // suppression latch NOT armed

  ArgAxisOnly all{}; all.axis = AXIS_ALL;
  CHECK(call(RpcMethod::REBOOT_ODRIVES, all) == RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);
  CHECK(g_reboot_calls == 0);

  // Once the stow clears, REBOOT proceeds normally (one leg frame, latch armed).
  g_stow_pending = false;
  CHECK(call(RpcMethod::REBOOT_ODRIVES, leg) == RpcStatus::OK);
  CHECK(fake_sent_count() == 1);
  CHECK(g_reboot_calls == 1);
}

// ── Flash-A item 6: CLEAR_ERRORS notifies (refills the soft-reset budget) ONLY ────
//    after the gate passes — a bus-down clear must not refill it.

TEST_CASE("item 6: a bus-down CLEAR_ERRORS does NOT refill the soft-reset budget (notify after gate)") {
  // AXIS_ALL, both gates closed (electrically dead bus) → ERR_BUS_DOWN, no notify, no TX.
  reset_all();
  fake_set_commands_allowed(false);
  g_bus_transmittable = false;
  ArgAxisOnly all{}; all.axis = AXIS_ALL;
  CHECK(call(RpcMethod::CLEAR_ERRORS, all) == RpcStatus::ERR_BUS_DOWN);
  CHECK(g_clear_calls == 0);           // the budget refill is GATED (was unconditional before)
  CHECK(fake_sent_count() == 0);

  // Per-axis, gated → also no notify (the per-axis path mirrors the AXIS_ALL discipline).
  reset_all();
  fake_set_commands_allowed(false);
  g_bus_transmittable = false;
  ArgAxisOnly one{}; one.axis = 0;
  CHECK(call(RpcMethod::CLEAR_ERRORS, one) == RpcStatus::ERR_BUS_DOWN);
  CHECK(g_clear_calls == 0);
  CHECK(fake_sent_count() == 0);

  // An up-bus (transmittable) AXIS_ALL clear DOES refill (gate passes → notify).
  reset_all();
  g_bus_transmittable = true;
  CHECK(call(RpcMethod::CLEAR_ERRORS, all) == RpcStatus::OK);
  CHECK(g_clear_calls == 1);
}

// ── Flash-A item 7: an AXIS_ALL broadcast surfaces a partial TX-enqueue failure ───
//    as ERR_TIMEOUT (previously discarded behind an unconditional OK); every axis
//    is still attempted.

TEST_CASE("item 7: a partial CLEAR_ERRORS AXIS_ALL TX failure surfaces as ERR_TIMEOUT (all axes attempted)") {
  reset_all();
  fake_set_send_fail_index(2);         // the 3rd can_jugglebot_send attempt fails (not recorded)
  ArgAxisOnly all{}; all.axis = AXIS_ALL;
  CHECK(call(RpcMethod::CLEAR_ERRORS, all) == RpcStatus::ERR_TIMEOUT);
  // All NUM_AXES sends were attempted; the failed one is not recorded → NUM_AXES-1 recorded.
  CHECK(fake_sent_count() == CanBridge::NUM_AXES - 1);
  CHECK(g_clear_calls == 1);           // the gate passed, so the budget still refilled (item 6 ordering intact)
}

TEST_CASE("item 7: a partial REBOOT_ODRIVES AXIS_ALL TX failure surfaces as ERR_TIMEOUT (all axes attempted)") {
  reset_all();
  fake_set_send_fail_index(4);         // the 5th can_jugglebot_send attempt fails
  ArgAxisOnly all{}; all.axis = AXIS_ALL;
  CHECK(call(RpcMethod::REBOOT_ODRIVES, all) == RpcStatus::ERR_TIMEOUT);
  CHECK(fake_sent_count() == CanBridge::NUM_AXES - 1);
  CHECK(g_reboot_calls == 1);          // the watchdog-suppression latch armed after the gate (unchanged)
}

// ── cold-start move routing (entry points stubbed) ───────────────────────────

TEST_CASE("HOME / ACTIVATE / DEACTIVATE route to their *_request with the axis") {
  reset_all();
  ArgAxisOnly h{}; h.axis = 3;
  CHECK(call(RpcMethod::HOME, h) == RpcStatus::OK);
  CHECK(g_homing_axis == 3);
  ArgAxisOnly ac{}; ac.axis = AXIS_ALL;
  CHECK(call(RpcMethod::ACTIVATE, ac) == RpcStatus::OK);
  CHECK(g_activate_axis == AXIS_ALL);
  CHECK(call(RpcMethod::DEACTIVATE, ac) == RpcStatus::OK);
  CHECK(g_deactivate_axis == AXIS_ALL);
}

// ── Ball Butler routing + presence gate + range-check ─────────────────────────

TEST_CASE("BB_THROW: range-check rejects before CAN1; in-range + BB present sends one CAN1 frame") {
  reset_all();
  fake_set_clock(1700000000000000ULL, 0);
  // Out-of-range speed → ERR_BAD_ARGS, no CAN1 traffic (even before the presence gate).
  ArgBbThrow bad{}; bad.yaw_rad = 0.0f; bad.pitch_rad = 0.5f; bad.speed_mps = 100.0f;
  bad.release_wall_us = 1700000000000000ULL + 2000000ULL;
  CHECK(call(RpcMethod::BB_THROW, bad) == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_bb_sent == 0);
  // In-range + BB present → one CAN1 frame on the THROW id.
  reset_all();
  fake_set_clock(1700000000000000ULL, 0);
  bb_state.heartbeat_seen = true; bb_state.heartbeat_stale = false;
  ArgBbThrow ok{}; ok.yaw_rad = 0.0f; ok.pitch_rad = 0.5f; ok.speed_mps = 3.0f;
  ok.release_wall_us = 1700000000000000ULL + 2000000ULL;
  CHECK(call(RpcMethod::BB_THROW, ok) == RpcStatus::OK);
  CHECK(g_bb_sent == 1);
  CHECK(g_bb_last.id == BallButlerCanId::THROW_CMD);
}

// FW 29 / PROTOCOL_VERSION 10: the release is ABSOLUTE and the bridge is its lead
// gate. The frame's bytes 6-7 are the release's epoch-ms low 16 bits — NOT
// receipt + anything — so a late RPC cannot move the release; and a release that
// is gone, too close for BB, or past BB's ±32.768 s decode is REFUSED by name with
// nothing on CAN1 (no silent "throw now").
static uint16_t call_bb_throw(const ArgBbThrow& a, int32_t& lead_us, uint16_t& res_len) {
  uint8_t result[64]; res_len = 0;
  const uint16_t st = Rpc::dispatch(RpcMethod::BB_THROW, reinterpret_cast<const uint8_t*>(&a),
                                    sizeof(a), result, res_len);
  lead_us = 0;
  if (res_len == sizeof(JbUdp::RpcArgs::ResultBbThrow)) memcpy(&lead_us, result, 4);
  return st;
}

TEST_CASE("BB_THROW absolute release: bytes 6-7 are the release epoch-ms, independent of receipt time") {
  const uint64_t T = 1700000000123000ULL;            // the release (us, integer ms)
  const uint16_t want = (uint16_t)((T / 1000ULL) & 0xFFFFULL);
  for (uint64_t receipt_lag_us : {2500000ULL, 2480000ULL, 2400000ULL}) {   // 20 / 100 ms late
    reset_all();
    bb_state.heartbeat_seen = true; bb_state.heartbeat_stale = false;
    fake_set_clock(T - receipt_lag_us, 0);
    ArgBbThrow a{}; a.yaw_rad = 0.1f; a.pitch_rad = 0.5f; a.speed_mps = 3.0f; a.release_wall_us = T;
    int32_t lead = 0; uint16_t rl = 0;
    CHECK(call_bb_throw(a, lead, rl) == RpcStatus::OK);
    CHECK(rl == sizeof(JbUdp::RpcArgs::ResultBbThrow));
    CHECK(lead == (int32_t)receipt_lag_us);
    REQUIRE(g_bb_sent == 1);
    uint16_t tt; memcpy(&tt, &g_bb_last.buf[6], 2);
    CHECK(tt == want);
  }
}

TEST_CASE("BB_THROW absolute release: PAST / TOO_SOON / TOO_FAR refused by name, nothing on CAN1") {
  const uint64_t now = 1700000000000000ULL;
  struct Row { int64_t lead_us; uint16_t want; };
  const Row rows[] = {
    {-1000,                                   RpcStatus::ERR_RELEASE_PAST},
    {0,                                       RpcStatus::ERR_RELEASE_PAST},
    {1,                                       RpcStatus::ERR_RELEASE_TOO_SOON},
    {(int64_t)BB_RELEASE_MIN_LEAD_US - 1,     RpcStatus::ERR_RELEASE_TOO_SOON},
    {(int64_t)BB_RELEASE_MIN_LEAD_US,         RpcStatus::OK},
    {(int64_t)BB_RELEASE_MAX_LEAD_US,         RpcStatus::OK},
    {(int64_t)BB_RELEASE_MAX_LEAD_US + 1,     RpcStatus::ERR_RELEASE_TOO_FAR},
    {40000000,                                RpcStatus::ERR_RELEASE_TOO_FAR},  // would wrap at BB
  };
  for (const Row& r : rows) {
    reset_all();
    bb_state.heartbeat_seen = true; bb_state.heartbeat_stale = false;
    fake_set_clock(now, 0);
    ArgBbThrow a{}; a.yaw_rad = 0.0f; a.pitch_rad = 0.5f; a.speed_mps = 3.0f;
    a.release_wall_us = (uint64_t)((int64_t)now + r.lead_us);
    int32_t lead = 0; uint16_t rl = 0;
    CHECK(call_bb_throw(a, lead, rl) == r.want);
    CHECK(lead == (int32_t)r.lead_us);              // the lead rides back on refusals too
    CHECK(g_bb_sent == (r.want == RpcStatus::OK ? 1 : 0));
  }
  // A release 0 (an unset field) against a synced epoch clock is PAST, never "now".
  reset_all();
  bb_state.heartbeat_seen = true; bb_state.heartbeat_stale = false;
  fake_set_clock(now, 0);
  ArgBbThrow z{}; z.yaw_rad = 0.0f; z.pitch_rad = 0.5f; z.speed_mps = 3.0f; z.release_wall_us = 0;
  int32_t lead = 0; uint16_t rl = 0;
  CHECK(call_bb_throw(z, lead, rl) == RpcStatus::ERR_RELEASE_PAST);
  CHECK(lead == INT32_MIN);                          // saturated, not wrapped
  CHECK(g_bb_sent == 0);
}

TEST_CASE("BB_THROW aim (speed 0) ignores release_wall_us and stamps the bridge's now") {
  reset_all();
  bb_state.heartbeat_seen = true; bb_state.heartbeat_stale = false;
  const uint64_t now = 1700000000777000ULL;
  fake_set_clock(now, 0);
  ArgBbThrow a{}; a.yaw_rad = 0.2f; a.pitch_rad = 0.6f; a.speed_mps = 0.0f; a.release_wall_us = 0;
  int32_t lead = 0; uint16_t rl = 0;
  CHECK(call_bb_throw(a, lead, rl) == RpcStatus::OK);
  CHECK(rl == 0);
  REQUIRE(g_bb_sent == 1);
  uint16_t tt; memcpy(&tt, &g_bb_last.buf[6], 2);
  CHECK(tt == (uint16_t)((now / 1000ULL) & 0xFFFFULL));
}

TEST_CASE("BB_RELOAD with BB absent → ERR_BUS_DOWN (presence gate, no CAN1)") {
  reset_all();
  bb_state.heartbeat_seen = false;    // BB not present
  CHECK(call_noarg(RpcMethod::BB_RELOAD) == RpcStatus::ERR_BUS_DOWN);
  CHECK(g_bb_sent == 0);
}

TEST_CASE("BB_CANCEL (FW 30): BB present → one payloadless CANCEL_CMD on CAN1; absent → ERR_BUS_DOWN, nothing sent") {
  reset_all();
  bb_state.heartbeat_seen = true; bb_state.heartbeat_stale = false;
  CHECK(call_noarg(RpcMethod::BB_CANCEL) == RpcStatus::OK);
  REQUIRE(g_bb_sent == 1);
  CHECK(g_bb_last.id == BallButlerCanId::CANCEL_CMD);
  CHECK(g_bb_last.len == 0);
  // The cancel must never outrank a throw queued beside it on CAN1: lower id
  // wins arbitration, so THROW_CMD < CANCEL_CMD keeps "throw then cancel" order.
  CHECK(BallButlerCanId::THROW_CMD < BallButlerCanId::CANCEL_CMD);

  reset_all();
  bb_state.heartbeat_seen = false;    // BB not present
  CHECK(call_noarg(RpcMethod::BB_CANCEL) == RpcStatus::ERR_BUS_DOWN);
  CHECK(g_bb_sent == 0);
}

TEST_CASE("HAND_FAN_SET (FW 41): routes typed args to the relay, refuses a short payload") {
  reset_all();
  JbUdp::RpcArgs::ArgHandFanSet a{};
  a.mode = 1;
  a.on_above_c = 30.0f;
  a.off_below_c = 28.5f;
  CHECK(call(RpcMethod::HAND_FAN_SET, a) == RpcStatus::OK);
  CHECK(g_hand_fan_calls == 1);
  CHECK(g_hand_fan_args.mode == 1);
  CHECK(g_hand_fan_args.on_above_c == doctest::Approx(30.0f));
  CHECK(g_hand_fan_args.off_below_c == doctest::Approx(28.5f));

  uint8_t result[64];
  uint16_t res_len = 0;
  CHECK(Rpc::dispatch(RpcMethod::HAND_FAN_SET, reinterpret_cast<const uint8_t*>(&a), sizeof(a) - 1,
                      result, res_len) == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_hand_fan_calls == 1);
}

TEST_CASE("PLATFORM_IMU_CTRL / _READ (FW 45): typed args reach the relay, Armed is passed in, never refused for it") {
  reset_all();
  JbUdp::RpcArgs::ArgPlatformImuCtrl c{};
  c.op = PlatformImuOp::RESUME; c.capture_id = 0x4A21;
  g_mpc_active = true;   // Armed: both IMU RPCs are allowed (owner, 2026-11-19)
  CHECK(call(RpcMethod::PLATFORM_IMU_CTRL, c) == RpcStatus::OK);
  CHECK(g_imu_ctrl_calls == 1);
  CHECK(g_imu_ctrl_args.op == PlatformImuOp::RESUME);
  CHECK(g_imu_ctrl_args.capture_id == 0x4A21);

  // FW 47: SET_ODR rides the same RPC and is allowed while Armed too (odr_khz in `reason`).
  JbUdp::RpcArgs::ArgPlatformImuCtrl s{};
  s.op = PlatformImuOp::SET_ODR; s.reason = 2;
  CHECK(call(RpcMethod::PLATFORM_IMU_CTRL, s) == RpcStatus::OK);
  CHECK(g_imu_ctrl_calls == 2);
  CHECK(g_imu_ctrl_args.op == PlatformImuOp::SET_ODR);
  CHECK(g_imu_ctrl_args.reason == 2);

  JbUdp::RpcArgs::ArgPlatformImuRead r{};
  r.capture_id = 0x4A21; r.first_seq = 0xFFF0; r.count = 11; r.frame_gap_10us = 40;
  CHECK(call(RpcMethod::PLATFORM_IMU_READ, r) == RpcStatus::OK);
  CHECK(g_imu_read_calls == 1);
  CHECK(g_imu_read_armed);                       // rpc.cpp samples fault_mpc_active()
  CHECK(g_imu_read_args.first_seq == 0xFFF0);
  CHECK(g_imu_read_args.count == 11);
  CHECK(g_imu_read_args.frame_gap_10us == 40);   // the clamp is the relay's, not dispatch's

  g_mpc_active = false;
  CHECK(call(RpcMethod::PLATFORM_IMU_READ, r) == RpcStatus::OK);
  CHECK_FALSE(g_imu_read_armed);

  uint8_t result[64];
  uint16_t res_len = 0;
  CHECK(Rpc::dispatch(RpcMethod::PLATFORM_IMU_CTRL, reinterpret_cast<const uint8_t*>(&c), sizeof(c) - 1,
                      result, res_len) == RpcStatus::ERR_BAD_ARGS);
  CHECK(Rpc::dispatch(RpcMethod::PLATFORM_IMU_READ, reinterpret_cast<const uint8_t*>(&r), sizeof(r) - 1,
                      result, res_len) == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_imu_ctrl_calls == 2);
  CHECK(g_imu_read_calls == 2);
}

TEST_CASE("WAVE_CMD (FW 33): args verbatim as one DLC-8 0x7E2 frame on the cone bus; gate/arg refusals") {
  reset_all();
  ArgWaveCmd a{}; a.desired = 1; a.cmd_seq = 0x2A; a.flags = 0x01; a.reserved = 0;
  a.session_id = 0x6A1B2C3Du;
  CHECK(call(RpcMethod::WAVE_CMD, a) == RpcStatus::OK);
  REQUIRE(g_cone_sent == 1);
  CHECK(g_cone_last.id == WaveCameraCanId::CMD);
  CHECK(g_cone_last.id == 0x7E2u);
  CHECK(g_cone_last.len == 8);
  CHECK(g_cone_last_cls == TxCls::RPC);
  const uint8_t want[8] = {0x01, 0x2A, 0x01, 0x00, 0x3D, 0x2C, 0x1B, 0x6A};
  for (int i = 0; i < 8; ++i) CHECK(g_cone_last.buf[i] == want[i]);
  CHECK(g_bb_sent == 0);                       // never touches CAN1

  // DEFERRED reached the wire (queued, in order) -> OK.
  reset_all(); g_cone_result = TxResult::DEFERRED;
  CHECK(call(RpcMethod::WAVE_CMD, a) == RpcStatus::OK);

  // No cone-bus partner: the presence gate refuses -> ERR_BUS_DOWN, nothing sent.
  reset_all(); g_cone_result = TxResult::FAILED;
  CHECK(call(RpcMethod::WAVE_CMD, a) == RpcStatus::ERR_BUS_DOWN);
  CHECK(g_cone_sent == 0);

  // Arg validation precedes the gate: desired > 1, reserved != 0, short args.
  reset_all();
  ArgWaveCmd bad = a; bad.desired = 2;
  CHECK(call(RpcMethod::WAVE_CMD, bad) == RpcStatus::ERR_BAD_ARGS);
  bad = a; bad.reserved = 1;
  CHECK(call(RpcMethod::WAVE_CMD, bad) == RpcStatus::ERR_BAD_ARGS);
  CHECK(call_noarg(RpcMethod::WAVE_CMD) == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_cone_sent == 0);
}

// ── Ball Butler firmware-over-CAN routing + gate sampling (FW 21) ─────────────

TEST_CASE("BB_FW_*: each method routes to its relay op with the gate sampled from bb_state") {
  reset_all();
  bb_state.heartbeat_seen = true; bb_state.heartbeat_stale = false;
  bb_state.state = BallButlerState::ERROR;
  ArgPlatformFwBegin b{}; b.image_len = 1234;
  CHECK(call(RpcMethod::BB_FW_BEGIN, b) == RpcStatus::OK);
  CHECK(g_bb_fw_method == RpcMethod::BB_FW_BEGIN);
  CHECK(g_bb_fw_gate.seen);
  CHECK(g_bb_fw_gate.fresh);
  CHECK(g_bb_fw_gate.state == BallButlerState::ERROR);

  // A stale-but-seen heartbeat samples as seen && !fresh (the mid-transfer
  // sector-erase stall the in-session ops must ride through).
  bb_state.heartbeat_stale = true;
  ArgPlatformFwData d{}; d.seq = 7; d.n = 5;
  CHECK(call(RpcMethod::BB_FW_DATA, d) == RpcStatus::OK);
  CHECK(g_bb_fw_method == RpcMethod::BB_FW_DATA);
  CHECK(g_bb_fw_gate.seen);
  CHECK_FALSE(g_bb_fw_gate.fresh);

  ArgPlatformFwVerify v{}; v.crc32 = 0xDEADBEEF;
  CHECK(call(RpcMethod::BB_FW_VERIFY, v) == RpcStatus::OK);
  CHECK(g_bb_fw_method == RpcMethod::BB_FW_VERIFY);
  CHECK(call_noarg(RpcMethod::BB_FW_COMMIT) == RpcStatus::OK);
  CHECK(g_bb_fw_method == RpcMethod::BB_FW_COMMIT);
  CHECK(call_noarg(RpcMethod::BB_FW_INFO) == RpcStatus::OK);
  CHECK(g_bb_fw_method == RpcMethod::BB_FW_INFO);
  CHECK(g_bb_fw_calls == 5);
  CHECK(g_bb_sent == 0);   // the stubs own the CAN1 send; rpc.cpp adds none
  bb_state.heartbeat_stale = false;          // leave bb_state as later cases expect it
  bb_state.state = BallButlerState::BOOT;
}

TEST_CASE("BB_FW_* with a short arg → ERR_BAD_ARGS, relay never reached") {
  reset_all();
  const uint8_t oneb[1] = {0};
  uint8_t result[64]; uint16_t res_len = 0;
  CHECK(Rpc::dispatch(RpcMethod::BB_FW_BEGIN, oneb, 1, result, res_len) == RpcStatus::ERR_BAD_ARGS);
  CHECK(Rpc::dispatch(RpcMethod::BB_FW_DATA, oneb, 1, result, res_len) == RpcStatus::ERR_BAD_ARGS);
  CHECK(Rpc::dispatch(RpcMethod::BB_FW_VERIFY, oneb, 1, result, res_len) == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_bb_fw_calls == 0);
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

// ═══════════════════════════════════════════════════════════════════════════
//  TRI-STATE TX (2026-08-24) — per-caller rulings at the RPC seam
// ═══════════════════════════════════════════════════════════════════════════
//  send_axis_frame() is the single enforcement point for BOTH the gate basis and
//  the TxCls, derived from the same method_gates_on_bus_transmittable() predicate
//  so no call site can drift them apart. These cases pin that coupling and the
//  retry-only-on-FAILED rule the safety frames keep.

TEST_CASE("an ordinary RPC frame that DEFERS acks OK and is charged to TxCls::RPC") {
  reset_all();
  fake_set_send_defer_all(true);         // no TX mailbox free: queued, transmits in order
  ArgAxisState a{}; a.axis = 2; a.state = 8;
  // The old `write() > 0` made this ERR_TIMEOUT, which the Jetson RPC client then
  // RETRIED on — re-sending a command whose first copy was already in the transmit
  // queue. DEFERRED = sent, so no retry is provoked.
  CHECK(call(RpcMethod::SET_AXIS_STATE, a) == RpcStatus::OK);
  REQUIRE(fake_sent_count() == 1);
  CHECK(fake_last_tx_class() == (int)TxCls::RPC);
  CHECK(fake_sent_count_cls(TxCls::SAFETY) == 0u);
}

TEST_CASE("CLEAR_ERRORS rides the SAFETY bucket, and a deferral still fires the notify") {
  reset_all();
  fake_set_send_defer_all(true);
  ArgAxisOnly a{}; a.axis = AXIS_ALL;

  CHECK(call(RpcMethod::CLEAR_ERRORS, a) == RpcStatus::OK);
  // Every axis frame is on the bus (deferral is a queue hop), and every one of them
  // is charged to the DEDICATED safety-deferral counter — not to the generic RPC
  // bucket, because a deferred recovery one-shot is a watch signal in its own right
  // and must never be averaged into ordinary RPC traffic.
  CHECK(fake_sent_count() == (size_t)CanBridge::NUM_AXES);
  CHECK(fake_sent_count_cls(TxCls::SAFETY) == (size_t)CanBridge::NUM_AXES);
  CHECK(fake_sent_count_cls(TxCls::RPC) == 0u);
  // The side effect fires: "send FIRST, notify only if at least one frame was
  // actually enqueued" always MEANT "reached the bus", and a deferred frame has.
  CHECK(g_clear_calls == 1);
}

TEST_CASE("safety frames retry only on FAILED: a refused bus withholds the notify") {
  reset_all();
  // The presence gate refusing is the ONE outcome in which nothing reaches the bus.
  // It must still be ERR_TIMEOUT (so the client retries) and must still withhold
  // the side effects — a clear that cannot reach the bus must NOT refill the
  // soft-reset budget or release the guard E-STOP latch.
  for (uint8_t i = 0; i < CanBridge::NUM_AXES; ++i) {
    reset_all();
    fake_set_send_fail_index(-1);
    // Fail every attempt by failing index 0 and re-arming per axis is awkward; drive
    // the unambiguous case instead: a single-axis clear whose one send FAILS.
    fake_set_send_fail_index(0);
    ArgAxisOnly one{}; one.axis = i;
    if (i >= CanBridge::NUM_LEGS && !JbUdp::hand_axis6_permitted(RpcMethod::CLEAR_ERRORS)) continue;
    CHECK(call(RpcMethod::CLEAR_ERRORS, one) == RpcStatus::ERR_TIMEOUT);
    CHECK(fake_sent_count() == 0);
    CHECK(g_clear_calls == 0);           // no side effect on a true failure
  }

  // And the mixed case: one axis fails, the rest defer. `any` is true, so the notify
  // fires (frames DID reach the bus), while `ok` is false, so the status is honest.
  reset_all();
  fake_set_send_defer_all(true);
  fake_set_send_fail_index(0);           // only the FIRST axis genuinely fails
  ArgAxisOnly all{}; all.axis = AXIS_ALL;
  CHECK(call(RpcMethod::CLEAR_ERRORS, all) == RpcStatus::ERR_TIMEOUT);
  CHECK(fake_sent_count() == (size_t)(CanBridge::NUM_AXES - 1));
  CHECK(g_clear_calls == 1);
}

TEST_CASE("the BB relay acks a DEFERRED CAN1 frame OK, charged to TxCls::RPC") {
  reset_all();
  bb_state.heartbeat_seen = true; bb_state.heartbeat_stale = false;
  g_bb_result = TxResult::DEFERRED;      // CAN1 mailbox busy: queued, transmits in order
  fake_set_clock(1700000000000000ULL, 0);
  ArgBbThrow a{}; a.yaw_rad = 0.0f; a.pitch_rad = 0.5f; a.speed_mps = 3.0f;
  a.release_wall_us = 1700000000000000ULL + 2000000ULL;

  CHECK(call(RpcMethod::BB_THROW, a) == RpcStatus::OK);
  CHECK(g_bb_sent == 1);
  CHECK(g_bb_last.id == BallButlerCanId::THROW_CMD);
  CHECK(g_bb_last_cls == TxCls::RPC);

  // A genuine CAN1 failure (the BB presence gate refusing inside can_bb_tx) is
  // still ERR_TIMEOUT, so the client still retries the one case that needs it.
  reset_all();
  bb_state.heartbeat_seen = true; bb_state.heartbeat_stale = false;
  fake_set_clock(1700000000000000ULL, 0);
  g_bb_send_ok = false;                  // → TxResult::FAILED
  CHECK(call(RpcMethod::BB_THROW, a) == RpcStatus::ERR_TIMEOUT);
}


// ── The 0x54 / 0x55 holes (FW 21, PROTOCOL_VERSION 7) ────────────────────────
//  HAND_TRAJ_CMD (0x54) and HAND_SOURCE_SET (0x55) were deleted with the
//  Platform stroke engine and the hand-mastery latch. Both ids are now HOLES
//  with no dispatch case, so a stale host's call must fall to the dispatcher's
//  unknown-method answer and send NOTHING on any bus. This test is the fence
//  that keeps a future method from being renumbered into either id: if one is,
//  this case starts returning OK and fails.
TEST_CASE("deleted hand methods 0x54 / 0x55 answer ERR_UNKNOWN_METHOD and touch no bus") {
  reset_all();
  uint8_t result[64]; uint16_t res_len = 0;
  const uint8_t args[8] = {0};
  for (uint16_t id : {(uint16_t)0x0054, (uint16_t)0x0055}) {
    res_len = 0;
    CHECK(Rpc::dispatch(id, args, sizeof(args), result, res_len)
          == RpcStatus::ERR_UNKNOWN_METHOD);
    CHECK(res_len == 0);
  }
  CHECK(fake_sent_count() == 0);
}


// ═══ FW 22: GET_HAND_TORQUE_SCALE — hand ODrive can.input_torque_scale readback ═══
static void set_hand_version(uint8_t maj, uint8_t min, uint8_t rev) {
  // Get_Version payload: [0]=protocol [1..3]=hw [4]=fw_major [5]=fw_minor [6]=fw_rev [7]=unreleased
  memset(g_hand_ver_raw, 0, 8);
  g_hand_ver_raw[4] = maj; g_hand_ver_raw[5] = min; g_hand_ver_raw[6] = rev;
  g_hand_ver_have = true;
}
static JbUdp::RpcArgs::ResultHandTorqueScale hts_call(uint16_t* status = nullptr) {
  uint8_t result[64]; uint16_t res_len = 0;
  const uint16_t st = Rpc::dispatch(RpcMethod::GET_HAND_TORQUE_SCALE, nullptr, 0, result, res_len);
  if (status) *status = st;
  JbUdp::RpcArgs::ResultHandTorqueScale r{};
  REQUIRE(res_len == sizeof(r));
  memcpy(&r, result, sizeof(r));
  return r;
}

TEST_CASE("GET_HAND_TORQUE_SCALE: no RxSdo until the hand's Get_Version proves the (board, fw) the endpoint id belongs to") {
  reset_all();
  uint16_t st = 0xFFFF;
  CHECK(hts_call(&st).state == 3);            // never received a version
  CHECK(st == RpcStatus::OK);
  set_hand_version(0, 6, 10);
  CHECK(hts_call().state == 3);               // wrong fw: 283 may name another register
  CHECK(fake_sent_count() == 0);
  CHECK(hts_call().age_ms == 0xFFFFFFFFu);
}

TEST_CASE("GET_HAND_TORQUE_SCALE: fires ONE RxSdo read of endpoint 283 on axis 6, then serves the cached reply") {
  reset_all();
  set_hand_version(0, 6, 11);
  auto r = hts_call();
  CHECK(r.state == 1);
  CHECK(r.reply_seq == 0u);
  REQUIRE(fake_sent_count() == 1);
  const auto expect = ODrive::encode_sdo_read(HAND_AXIS, EndpointId::odrive_pro_0_6_11::can_input_torque_scale);
  CHECK(EndpointId::odrive_pro_0_6_11::can_input_torque_scale == 283);
  CHECK(fake_sent_at(0).id == expect.id);
  CHECK(memcmp(fake_sent_at(0).buf, expect.buf, 8) == 0);
  CHECK(hts_call().state == 1);               // in flight: no second RxSdo
  CHECK(fake_sent_count() == 1);
  fake_advance(3000);
  Rpc::hand_torque_scale_record(1000u, micros64());
  fake_advance(2000);
  r = hts_call();                             // cached → serves it AND triggers the next read
  CHECK(r.value == 1000u);
  CHECK(r.reply_seq == 1u);
  CHECK(r.age_ms == 2u);
  CHECK(r.state == 1);
  CHECK(fake_sent_count() == 2);
}

TEST_CASE("GET_HAND_TORQUE_SCALE: an unanswered read re-triggers after 500 ms; a refused bus reports state 4") {
  reset_all();
  set_hand_version(0, 6, 11);
  CHECK(hts_call().state == 1);
  fake_advance(499000);
  CHECK(hts_call().state == 1);
  CHECK(fake_sent_count() == 1);
  fake_advance(2000);
  CHECK(hts_call().state == 1);
  CHECK(fake_sent_count() == 2);
  reset_all();
  set_hand_version(0, 6, 11);
  fake_set_commands_allowed(false);
  CHECK(hts_call().state == 4);
  CHECK(fake_sent_count() == 0);
}

// ── HAND_MOVE_TO (FW 26): routing + the deferred reply ─────────────────────────
//  The validation itself is leg_activate's (test_leg_activate.cpp); here: the
//  args reach hand_move_request intact with THIS request's req_id, an accepted
//  move sends NOTHING now, a refusal answers now, and rpc_service sends the
//  posted reply as an RPC_RESPONSE for HAND_MOVE_TO with the echoed req_id.
static void send_request(uint16_t method, uint16_t req_id, const void* args, uint16_t n) {
  uint8_t buf[64];
  const uint16_t len = Rpc::pack_request(method, req_id, static_cast<const uint8_t*>(args), n,
                                         buf, sizeof(buf));
  REQUIRE(len > 0);
  Rpc::on_request(0, buf, len);
}

TEST_CASE("HAND_MOVE_TO: args + req_id reach hand_move_request; an accepted move defers its reply") {
  reset_all();
  g_udp_sends = 0; g_hm_ret = RpcStatus::OK; g_hm_reply_ready = false;
  JbUdp::RpcArgs::ArgHandMoveTo a{};
  a.axis = HAND_AXIS; a.target_rev = 3.25f; a.vel_rps = 1.5f;
  send_request(RpcMethod::HAND_MOVE_TO, 0x1234, &a, sizeof(a));
  CHECK(g_hm_axis == HAND_AXIS);
  CHECK(g_hm_target == doctest::Approx(3.25f));
  CHECK(g_hm_vel == doctest::Approx(1.5f));
  CHECK(g_hm_req_id == 0x1234);
  CHECK(g_udp_sends == 0);          // deferred: no reply until the move ends

  // The activate task posts the terminal reply; rpc_service sends it.
  g_hm_reply.req_id = 0x1234; g_hm_reply.status = RpcStatus::OK;
  memset(&g_hm_reply.result, 0, sizeof(g_hm_reply.result));
  g_hm_reply.result.outcome = HAND_MOVE_ARRIVED; g_hm_reply.result.pos_rev = 3.249f;
  g_hm_reply_ready = true;
  Rpc::rpc_service();
  REQUIRE(g_udp_sends == 1);
  CHECK(g_udp_last_type == JbUdp::MsgType::RPC_RESPONSE);
  uint16_t m = 0, id = 0, st = 0, rl = 0; const uint8_t* res = nullptr;
  REQUIRE(Rpc::parse_response(g_udp_last, g_udp_last_len, &m, &id, &st, &res, &rl));
  CHECK(m == RpcMethod::HAND_MOVE_TO);
  CHECK(id == 0x1234);
  CHECK(st == RpcStatus::OK);
  REQUIRE(rl == sizeof(JbUdp::RpcArgs::ResultHandMoveTo));
  JbUdp::RpcArgs::ResultHandMoveTo r; memcpy(&r, res, sizeof(r));
  CHECK(r.outcome == HAND_MOVE_ARRIVED);
  CHECK(r.pos_rev == doctest::Approx(3.249f));
  Rpc::rpc_service();
  CHECK(g_udp_sends == 1);          // drained: nothing more to send
}

TEST_CASE("HAND_MOVE_TO: a refusal answers immediately with the refusal status") {
  reset_all();
  g_udp_sends = 0; g_hm_ret = RpcStatus::ERR_BUS_DOWN; g_hm_reply_ready = false;
  JbUdp::RpcArgs::ArgHandMoveTo a{};
  a.axis = HAND_AXIS; a.target_rev = 1.0f; a.vel_rps = 2.5f;
  send_request(RpcMethod::HAND_MOVE_TO, 7, &a, sizeof(a));
  REQUIRE(g_udp_sends == 1);
  uint16_t m = 0, id = 0, st = 0, rl = 0; const uint8_t* res = nullptr;
  REQUIRE(Rpc::parse_response(g_udp_last, g_udp_last_len, &m, &id, &st, &res, &rl));
  CHECK(id == 7);
  CHECK(st == RpcStatus::ERR_BUS_DOWN);
  g_hm_ret = RpcStatus::OK;
}

TEST_CASE("HAND_MOVE_TO: a short arg blob is ERR_BAD_ARGS and never reaches the op") {
  reset_all();
  g_hm_axis = 0xEE;
  uint8_t oneb[1] = {HAND_AXIS};
  uint8_t result[64]; uint16_t res_len = 0;
  CHECK(Rpc::dispatch(RpcMethod::HAND_MOVE_TO, oneb, 1, result, res_len) == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_hm_axis == 0xEE);
}


// ── ACTIVATE_TO (FW 51, BUG-130): the six targets reach the one enforcement point ──
TEST_CASE("ACTIVATE_TO: the six targets reach activate_to_request intact; its status is the reply; a short arg never calls it") {
  reset_all();
  g_activate_to_calls = 0; g_activate_to_ret = RpcStatus::OK;
  JbUdp::RpcArgs::ArgActivateTo a{};
  for (int i = 0; i < 6; ++i) a.target_rev[i] = 2.1f + 0.01f * (float)i;
  send_request(RpcMethod::ACTIVATE_TO, 0x0042, &a, sizeof(a));
  REQUIRE(g_activate_to_calls == 1);
  for (int i = 0; i < 6; ++i) CHECK(g_activate_to_targets[i] == a.target_rev[i]);
  uint16_t m = 0, id = 0, st = 0, rl = 0; const uint8_t* res = nullptr;
  REQUIRE(Rpc::parse_response(g_udp_last, g_udp_last_len, &m, &id, &st, &res, &rl));
  CHECK(m == RpcMethod::ACTIVATE_TO);
  CHECK(st == RpcStatus::OK);

  g_activate_to_ret = RpcStatus::ERR_BAD_ARGS;   // the band refusal passes straight through
  send_request(RpcMethod::ACTIVATE_TO, 0x0043, &a, sizeof(a));
  REQUIRE(Rpc::parse_response(g_udp_last, g_udp_last_len, &m, &id, &st, &res, &rl));
  CHECK(st == RpcStatus::ERR_BAD_ARGS);

  g_activate_to_calls = 0;
  send_request(RpcMethod::ACTIVATE_TO, 0x0044, &a, sizeof(a) - 4);   // five and a bit
  CHECK(g_activate_to_calls == 0);
  REQUIRE(Rpc::parse_response(g_udp_last, g_udp_last_len, &m, &id, &st, &res, &rl));
  CHECK(st == RpcStatus::ERR_BAD_ARGS);
}

// ═══════════════════════════════════════════════════════════════════════════
//  FW 52 (BUG-99 D1, row 4a): reconfiguration RPCs refused while Armed. One
//  predicate (refused_while_armed) at one enforcement point (send_axis_frame).
// ═══════════════════════════════════════════════════════════════════════════
using JbUdp::RpcArgs::ArgControllerMode;
using JbUdp::RpcArgs::ArgVelCurr;
using JbUdp::RpcArgs::ArgPosGain;
using JbUdp::RpcArgs::ArgVelGains;
using JbUdp::RpcArgs::ArgSdoWrite;

// Every method on the decision list, on a Leg (0) and on the Hand (6), Armed and
// not: the status, and whether a frame reached CAN.
static void check_method(uint16_t method, uint8_t axis, uint32_t state, bool armed, uint16_t want) {
  reset_all();
  g_mpc_active = armed;
  uint16_t st = 0xEEEE;
  switch (method) {
    case RpcMethod::SET_AXIS_STATE: { ArgAxisState a{}; a.axis = axis; a.state = state; st = call(method, a); break; }
    case RpcMethod::SET_CONTROLLER_MODE: { ArgControllerMode a{}; a.axis = axis; a.ctrl = 3; a.input = 1; st = call(method, a); break; }
    case RpcMethod::SET_VEL_CURR_LIMITS: { ArgVelCurr a{}; a.axis = axis; a.vel_limit = 5.0f; a.curr_limit = 10.0f; st = call(method, a); break; }
    case RpcMethod::SET_POS_GAIN: { ArgPosGain a{}; a.axis = axis; a.pos_gain = 40.0f; st = call(method, a); break; }
    case RpcMethod::SET_VEL_GAINS: { ArgVelGains a{}; a.axis = axis; a.vel_gain = 0.1f; a.vel_int_gain = 0.2f; st = call(method, a); break; }
    case RpcMethod::SET_ABSOLUTE_POSITION: { ArgAbsPosition a{}; a.axis = axis; a.position = 1.0f; st = call(method, a); break; }
    case RpcMethod::SDO_WRITE: { ArgSdoWrite a{}; a.axis = axis; a.endpoint = 265; a.value = 1.0f; st = call(method, a); break; }
    case RpcMethod::CLEAR_ERRORS: { ArgAxisOnly a{}; a.axis = axis; st = call(method, a); break; }
    default: FAIL("unexpected method");
  }
  INFO("method 0x" << std::hex << method << std::dec << " axis " << (int)axis << " state " << state << " armed " << armed);
  CHECK(st == want);
  CHECK(fake_sent_count() == (want == RpcStatus::OK ? 1 : 0));
}

TEST_CASE("FW 52 D1: while Armed the reconfiguration RPCs are ERR_REJECTED with nothing on CAN; disarmed they pass") {
  const uint32_t IDLE = ODriveState::IDLE, CL = ODriveState::CLOSED_LOOP;
  // Refused while Armed, on every axis (SDO_WRITE is not in the Hand allow-table at
  // all, so it is checked on the Leg only).
  for (uint8_t axis : {uint8_t(0), uint8_t(5), uint8_t(HAND_AXIS)}) {
    for (uint16_t m : {RpcMethod::SET_ABSOLUTE_POSITION, RpcMethod::SET_CONTROLLER_MODE, RpcMethod::SET_VEL_GAINS}) {
      check_method(m, axis, 0, true,  RpcStatus::ERR_REJECTED);
      check_method(m, axis, 0, false, RpcStatus::OK);
    }
    check_method(RpcMethod::SET_AXIS_STATE, axis, CL, true,  RpcStatus::ERR_REJECTED);
    check_method(RpcMethod::SET_AXIS_STATE, axis, CL, false, RpcStatus::OK);
    check_method(RpcMethod::SET_AXIS_STATE, axis, ODriveState::ENCODER_INDEX_SEARCH, true, RpcStatus::ERR_REJECTED);
    // Kept while Armed: IDLE (a stop), the limits (the jam relief, the GUI) and CLEAR_ERRORS.
    check_method(RpcMethod::SET_AXIS_STATE, axis, IDLE, true, RpcStatus::OK);
    check_method(RpcMethod::SET_VEL_CURR_LIMITS, axis, 0, true, RpcStatus::OK);
    check_method(RpcMethod::CLEAR_ERRORS, axis, 0, true, RpcStatus::OK);
  }
  check_method(RpcMethod::SDO_WRITE, 0, 0, true,  RpcStatus::ERR_REJECTED);
  check_method(RpcMethod::SDO_WRITE, 0, 0, false, RpcStatus::OK);
  // SET_POS_GAIN: refused on a Leg while Armed; the Hand's stays allowed (the
  // owner's decision names the Leg: Kp x MAX_LEAD_REV is a Leg invariant).
  check_method(RpcMethod::SET_POS_GAIN, 0, 0, true,  RpcStatus::ERR_REJECTED);
  check_method(RpcMethod::SET_POS_GAIN, 5, 0, true,  RpcStatus::ERR_REJECTED);
  check_method(RpcMethod::SET_POS_GAIN, 0, 0, false, RpcStatus::OK);
  check_method(RpcMethod::SET_POS_GAIN, HAND_AXIS, 0, true, RpcStatus::OK);
  // The AXIS_ALL clear stays allowed while Armed (/recover).
  reset_all(); g_mpc_active = true;
  ArgAxisOnly all{}; all.axis = AXIS_ALL;
  CHECK(call(RpcMethod::CLEAR_ERRORS, all) == RpcStatus::OK);
  CHECK(g_clear_calls == 1);
}

TEST_CASE("FW 52 D1: the gate keys on ARMED, so the real arming order passes (preamble before mpc_active)") {
  // teensy_bridge_node._arm_setpoint_output_locked: the stream is already flowing
  // (the pre-check), then Hand SET_AXIS_STATE CLOSED_LOOP and SET_CONTROLLER_MODE
  // POSITION/PASSTHROUGH, THEN _set_mpc_active(True). The bridge's fault_mpc_active()
  // follows the heartbeat bit, so it is still false at the preamble.
  reset_all();
  g_mpc_active = false;                       // frames may be streaming; not Armed yet
  ArgAxisState cl{}; cl.axis = HAND_AXIS; cl.state = ODriveState::CLOSED_LOOP;
  ArgControllerMode cm{}; cm.axis = HAND_AXIS; cm.ctrl = 3; cm.input = 1;
  CHECK(call(RpcMethod::SET_AXIS_STATE, cl) == RpcStatus::OK);
  CHECK(call(RpcMethod::SET_CONTROLLER_MODE, cm) == RpcStatus::OK);
  CHECK(fake_sent_count() == 2);
  g_mpc_active = true;                        // the heartbeat with bit 0 lands: Armed
  fake_clear_sent();
  CHECK(call(RpcMethod::SET_AXIS_STATE, cl) == RpcStatus::ERR_REJECTED);
  CHECK(call(RpcMethod::SET_CONTROLLER_MODE, cm) == RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);
  // The disarm path (Hand IDLE) is never refused.
  ArgAxisState idle{}; idle.axis = HAND_AXIS; idle.state = ODriveState::IDLE;
  CHECK(call(RpcMethod::SET_AXIS_STATE, idle) == RpcStatus::OK);
  CHECK(fake_sent_count() == 1);
}

TEST_CASE("FW 52 D2: only the AXIS_ALL clear calls the global release; a per-axis clear clears that axis") {
  for (uint8_t axis : {uint8_t(0), uint8_t(3), uint8_t(HAND_AXIS)}) {
    reset_all();
    ArgAxisOnly one{}; one.axis = axis;
    CHECK(call(RpcMethod::CLEAR_ERRORS, one) == RpcStatus::OK);
    CHECK(g_clear_calls == 0);
    CHECK(g_axis_clear_calls == 1);
    CHECK(g_axis_clear_axis == axis);
  }
  reset_all();
  ArgAxisOnly all{}; all.axis = AXIS_ALL;
  CHECK(call(RpcMethod::CLEAR_ERRORS, all) == RpcStatus::OK);
  CHECK(g_clear_calls == 1);
  CHECK(g_axis_clear_calls == 0);
  // A per-axis clear that never reached the wire clears nothing.
  reset_all();
  fake_set_send_fail_index(0);
  ArgAxisOnly z{}; z.axis = 0;
  CHECK(call(RpcMethod::CLEAR_ERRORS, z) == RpcStatus::ERR_TIMEOUT);
  CHECK(g_axis_clear_calls == 0);
}

TEST_CASE("FW 52 D1: REBOOT_ODRIVES is refused while Armed, per axis AND the AXIS_ALL broadcast (no TX, no latch); SDO_READ stays allowed") {
  // Orchestrator decision 2026-11-25, the owner's D1 intent: a reboot drops a live
  // loop mid-stream. The AXIS_ALL broadcast bypasses send_axis_frame, so it is
  // checked on its own (the second call site of the predicate).
  for (uint8_t axis : {uint8_t(0), uint8_t(5), uint8_t(HAND_AXIS), uint8_t(AXIS_ALL)}) {
    reset_all();
    g_mpc_active = true;
    ArgAxisOnly a{}; a.axis = axis;
    INFO("axis " << (int)axis);
    CHECK(call(RpcMethod::REBOOT_ODRIVES, a) == RpcStatus::ERR_REJECTED);
    CHECK(fake_sent_count() == 0);
    CHECK(g_reboot_calls == 0);            // the watchdog-suppression latch never armed
    g_mpc_active = false;                  // disarmed: the same reboot goes out
    CHECK(call(RpcMethod::REBOOT_ODRIVES, a) == RpcStatus::OK);
    CHECK(fake_sent_count() == (axis == AXIS_ALL ? (size_t)CanBridge::NUM_AXES : 1u));
  }
  reset_all();
  g_mpc_active = true;
  ArgSdoRead r{}; r.axis = 0; r.endpoint = 265;
  CHECK(call(RpcMethod::SDO_READ, r) == RpcStatus::OK);
  CHECK(fake_sent_count() == 1);
}


// ═════════════════════════════════════════════════════════════════════════════
// FW 53 U5 (I-FW-19, fw53_design.md 4): the RPC layer's de-energise requests go
// through the Stow gate. Owner rule: "A Leg should never be made IDLE unless the
// Platform is already at the Stow position." The Hand is exempt; so is the one-Leg
// bench (rig_is_bench). Refusals are the EXISTING ERR_REJECTED with nothing on CAN.
// ═════════════════════════════════════════════════════════════════════════════

static constexpr uint32_t ST_IDLE = 1, ST_ENC_SEARCH = 6, ST_CL = 8;
static constexpr uint64_t T0_US = 100'000'000ull;    // a clock far from zero: ages are now - age

// Stage Leg i as a PRESENT Leg reporting `st`, its heartbeat and encoder sample 1 ms
// old (a fixed literal: fresh under every threshold, never derived from one).
static void stage_leg(uint8_t i, uint8_t st, float pos, float vel) {
  axes[i].heartbeat_seen = true;
  axes[i].active_errors = 0;
  axes[i].axis_state = st;
  axes[i].last_heartbeat_us = fake_mono_us() - 1000;
  axes[i].last_rx_us = fake_mono_us() - 1000;
  write_pos_vel(axes[i], pos, vel, fake_mono_us() - 1000);
}
// Six IDLE Legs at the hardstop (the Platform at Stow, what Homing leaves).
static void stage_stowed() {
  for (uint8_t i = 0; i < CanBridge::NUM_LEGS; ++i) stage_leg(i, ST_IDLE, -0.1f, 0.0f);
}
// The Platform posed: Leg `posed` Closed-loop at 2.2 rev, the other five IDLE.
static void stage_posed(uint8_t posed) {
  stage_stowed();
  stage_leg(posed, ST_CL, 2.2f, 0.0f);
}
// Every axis back to absent (axes[] is a process global that earlier cases stage), the
// gate's file-statics zeroed (reset_all), and a clock far from zero.
static void fw53_reset() {
  reset_all();
  for (uint8_t i = 0; i < CanBridge::NUM_AXES; ++i) {
    axes[i].heartbeat_seen = false;
    axes[i].last_heartbeat_us = 0;
    axes[i].last_rx_us = 0;
    axes[i].axis_state = 0;
    axes[i].active_errors = 0;
    write_pos_vel(axes[i], 0.0f, 0.0f, 0);
  }
  fake_set_clock(T0_US, T0_US);
}

static uint16_t set_state(uint8_t axis, uint32_t st) {
  ArgAxisState a{}; a.axis = axis; a.state = st;
  return call(RpcMethod::SET_AXIS_STATE, a);
}
static uint16_t reboot(uint8_t axis) { ArgAxisOnly a{}; a.axis = axis; return call(RpcMethod::REBOOT_ODRIVES, a); }
static uint16_t sdo_write(uint8_t axis) {
  JbUdp::RpcArgs::ArgSdoWrite a{}; a.axis = axis; a.endpoint = 265; a.value = 1.0f;
  return call(RpcMethod::SDO_WRITE, a);
}

TEST_CASE("FW 53: the classifiers (twins of teensy_link/rpc_args.py) name exactly the Leg de-energise requests") {
  using Rpc::requires_platform_at_stow; using Rpc::home_requires_all_legs_idle;
  for (uint8_t leg = 0; leg < CanBridge::NUM_LEGS; ++leg) {
    INFO("leg " << (int)leg);
    CHECK(requires_platform_at_stow(RpcMethod::SET_AXIS_STATE, leg, ST_IDLE));
    CHECK(requires_platform_at_stow(RpcMethod::SET_AXIS_STATE, leg, ST_ENC_SEARCH));
    CHECK_FALSE(requires_platform_at_stow(RpcMethod::SET_AXIS_STATE, leg, ST_CL));   // engaging is not de-energising
    CHECK(requires_platform_at_stow(RpcMethod::REBOOT_ODRIVES, leg, 0));
    CHECK(requires_platform_at_stow(RpcMethod::SDO_WRITE, leg, 0));
    CHECK_FALSE(requires_platform_at_stow(RpcMethod::SDO_READ, leg, 0));
    CHECK_FALSE(requires_platform_at_stow(RpcMethod::SET_VEL_CURR_LIMITS, leg, 0));
    CHECK_FALSE(requires_platform_at_stow(RpcMethod::CLEAR_ERRORS, leg, 0));
    CHECK(home_requires_all_legs_idle(leg));
  }
  CHECK_FALSE(requires_platform_at_stow(RpcMethod::SET_AXIS_STATE, HAND_AXIS, ST_IDLE));   // the Hand is exempt
  CHECK_FALSE(requires_platform_at_stow(RpcMethod::REBOOT_ODRIVES, HAND_AXIS, 0));
  CHECK(requires_platform_at_stow(RpcMethod::REBOOT_ODRIVES, AXIS_ALL, 0));
  CHECK_FALSE(requires_platform_at_stow(RpcMethod::SET_AXIS_STATE, AXIS_ALL, ST_IDLE));    // not a valid broadcast
  CHECK_FALSE(home_requires_all_legs_idle(HAND_AXIS));
}

TEST_CASE("FW 53: SET_AXIS_STATE(Leg, IDLE) is ERR_REJECTED with nothing on CAN unless the Platform is at Stow; the Hand always passes") {
  fw53_reset();
  stage_stowed();
  CHECK(set_state(2, ST_IDLE) == RpcStatus::OK);                       // at Stow: the FW 52 behaviour
  REQUIRE(fake_sent_count() == 1);
  const auto f = ODrive::encode_set_state(2, ST_IDLE);
  CHECK(fake_sent_at(0).id == f.id);
  CHECK(memcmp(fake_sent_at(0).buf, f.buf, 8) == 0);

  for (uint8_t target : {uint8_t(0), uint8_t(2), uint8_t(5)}) {          // the target and two other Legs
    fw53_reset();
    stage_posed(3);                                                     // Leg 3 holds 2.2 rev
    INFO("IDLE to leg " << (int)target << " with leg 3 posed");
    CHECK(set_state(target, ST_IDLE) == RpcStatus::ERR_REJECTED);       // every Leg is refused, not only Leg 3
    CHECK(fake_sent_count() == 0);
    CHECK(leg_deenergise_refusals() == 1);                              // counted ...
    CHECK(leg_held_mask() == 0);                                        // ... but an RPC refusal is not a HELD Leg
  }
  fw53_reset();
  stage_posed(3);
  CHECK(set_state(HAND_AXIS, ST_IDLE) == RpcStatus::OK);                // Hand IDLE always passes
  CHECK(fake_sent_count() == 1);
}

TEST_CASE("FW 53: a Leg IDLE while Armed passes the Armed gate and is then refused by the Stow gate; at Stow it still passes") {
  fw53_reset();
  g_mpc_active = true;
  stage_posed(1);
  CHECK(set_state(4, ST_IDLE) == RpcStatus::ERR_REJECTED);              // Armed implies posed
  CHECK(fake_sent_count() == 0);
  CHECK(leg_deenergise_refusals() == 1);                                // proof it got PAST the Armed gate to the Stow gate
  CHECK(set_state(HAND_AXIS, ST_IDLE) == RpcStatus::OK);                // the Hand is exempt, Armed or not
  fw53_reset();
  g_mpc_active = true;
  stage_stowed();
  CHECK(set_state(4, ST_IDLE) == RpcStatus::OK);                        // a stop at Stow is still a stop
}

TEST_CASE("FW 53: SET_AXIS_STATE(Leg, 6) passes with every Leg IDLE and is refused with one posed") {
  fw53_reset();
  stage_stowed();
  CHECK(set_state(0, ST_ENC_SEARCH) == RpcStatus::OK);                  // the production encoder-search path
  CHECK(fake_sent_count() == 1);
  fw53_reset();
  stage_posed(5);
  CHECK(set_state(0, ST_ENC_SEARCH) == RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);
}

TEST_CASE("FW 53: a Leg CLOSED_LOOP is refused while a Homing runs and passes otherwise (it is not gated); the Hand CL is never refused") {
  fw53_reset();
  stage_posed(3);                                                       // posed: engaging another Leg is not a de-energise
  CHECK(set_state(0, ST_CL) == RpcStatus::OK);
  CHECK(fake_sent_count() == 1);
  fw53_reset();
  stage_stowed();
  fake_set_homing(true);
  CHECK(set_state(0, ST_CL) == RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);
  CHECK(leg_cl_cmd_us(0) == 0);                                         // no CL stamp left behind by the refusal
  CHECK(set_state(HAND_AXIS, ST_CL) == RpcStatus::OK);                  // the Hand CL preamble is untouched
  CHECK(fake_sent_count() == 1);
  fake_set_homing(false);
  CHECK(set_state(0, ST_CL) == RpcStatus::OK);
}

TEST_CASE("FW 53 BUG-47: the Leg a Homing is driving may be IDLEd (HOMING_OWNER), no other Leg may") {
  fw53_reset();
  stage_posed(4);
  fake_set_homing(true);
  fake_set_homing_axis(2);
  CHECK(set_state(2, ST_IDLE) == RpcStatus::OK);                        // the homing-abort IDLE
  REQUIRE(fake_sent_count() == 1);
  const auto f = ODrive::encode_set_state(2, ST_IDLE);
  CHECK(fake_sent_at(0).id == f.id);
  CHECK(set_state(3, ST_IDLE) == RpcStatus::ERR_REJECTED);              // not the in-flight Leg
  CHECK(set_state(2, ST_ENC_SEARCH) == RpcStatus::ERR_REJECTED);        // the carve-out is IDLE only
  CHECK(fake_sent_count() == 1);
  fake_set_homing_axis(0xFF);
  CHECK(set_state(2, ST_IDLE) == RpcStatus::ERR_REJECTED);              // no Homing in flight: no carve-out
}

TEST_CASE("FW 53: REBOOT per Leg and AXIS_ALL are refused WHOLE when the Platform is posed (no Leg frame, no Hand frame, latch not armed)") {
  fw53_reset();
  stage_posed(3);
  CHECK(reboot(3) == RpcStatus::ERR_REJECTED);
  CHECK(reboot(0) == RpcStatus::ERR_REJECTED);                          // an IDLE Leg too: the loop of the posed one drops
  CHECK(reboot(AXIS_ALL) == RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);                                        // not even the Hand's frame
  CHECK(g_reboot_calls == 0);                                           // the watchdog-suppression latch was never armed
  CHECK(reboot(HAND_AXIS) == RpcStatus::OK);                            // Hand-only reboot is unchanged
  CHECK(fake_sent_count() == 1);
  CHECK(g_reboot_calls == 0);

  fw53_reset();
  stage_stowed();                                                       // at Stow everything is as FW 52
  CHECK(reboot(AXIS_ALL) == RpcStatus::OK);
  CHECK(fake_sent_count() == (size_t)CanBridge::NUM_AXES);
  CHECK(g_reboot_calls == 1);
  fw53_reset();
  stage_stowed();
  CHECK(reboot(1) == RpcStatus::OK);
  CHECK(fake_sent_count() == 1);
  CHECK(g_reboot_calls == 1);
}

TEST_CASE("FW 53: SDO_WRITE(Leg) is gated, and refused while a Homing runs even at Stow; the Hand stays refused by its allow-table") {
  fw53_reset();
  stage_posed(2);
  CHECK(sdo_write(2) == RpcStatus::ERR_REJECTED);
  CHECK(sdo_write(0) == RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);
  fw53_reset();
  stage_stowed();
  CHECK(sdo_write(2) == RpcStatus::OK);
  CHECK(fake_sent_count() == 1);
  fake_set_homing(true);
  CHECK(sdo_write(2) == RpcStatus::ERR_REJECTED);                       // an SDO write can set requested_state
  CHECK(fake_sent_count() == 1);
  CHECK(sdo_write(HAND_AXIS) == RpcStatus::ERR_REJECTED);               // unchanged (hand_axis6_permitted)
}

TEST_CASE("FW 53: the wrapper's TX_FAILED is the only ERR_TIMEOUT (a refused gate is ERR_REJECTED, not a retry basis)") {
  fw53_reset();
  stage_stowed();
  fake_set_send_fail_index(0);                                          // no bus partner: nothing reaches the wire
  CHECK(set_state(1, ST_IDLE) == RpcStatus::ERR_TIMEOUT);
  fw53_reset();
  stage_stowed();
  fake_set_send_defer_all(true);                                        // queued, transmits in order
  CHECK(set_state(1, ST_IDLE) == RpcStatus::OK);
  CHECK(sdo_write(1) == RpcStatus::OK);
  CHECK(reboot(1) == RpcStatus::OK);
}

TEST_CASE("FW 53: CLEAR_ERRORS(AXIS_ALL) clears every Leg's HELD bit; a per-axis clear and a dead bus do not") {
  fw53_reset();
  stage_posed(1);
  (void)CanBridge::leg_freeze_tx(1, -0.11f, 2.5f);                      // an aborted ladder's hold
  (void)CanBridge::leg_freeze_tx(4, -0.11f, 2.5f);
  REQUIRE(leg_held_mask() == ((1u << 1) | (1u << 4)));
  ArgAxisOnly one{}; one.axis = 1;
  CHECK(call(RpcMethod::CLEAR_ERRORS, one) == RpcStatus::OK);
  CHECK(leg_held_mask() == ((1u << 1) | (1u << 4)));                    // per-axis: releases no latch, clears no HELD
  g_bus_transmittable = false;
  ArgAxisOnly all{}; all.axis = AXIS_ALL;
  CHECK(call(RpcMethod::CLEAR_ERRORS, all) == RpcStatus::ERR_BUS_DOWN);
  CHECK(leg_held_mask() == ((1u << 1) | (1u << 4)));                    // nothing reached the bus: not an acknowledgement
  g_bus_transmittable = true;
  CHECK(call(RpcMethod::CLEAR_ERRORS, all) == RpcStatus::OK);
  CHECK(leg_held_mask() == 0);
}

TEST_CASE("FW 53 one-Leg bench (rig_note_* staged): a Leg IDLE, reboot and SDO write pass at 2 rev after the 5 s settle, and not before") {
  fw53_reset();
  const uint64_t t0 = fake_mono_us();
  CanBridge::rig_note_bus(true, t0);
  CanBridge::rig_note_axis_frame(0, t0);                                // exactly one ODrive, no Platform frame
  stage_leg(0, ST_CL, 2.0f, 0.0f);                                      // the bench Leg, Closed-loop 2 rev off Stow
  fake_advance(2'000'000);                                              // 2 s: inside the settle window
  stage_leg(0, ST_CL, 2.0f, 0.0f);
  CHECK(set_state(0, ST_IDLE) == RpcStatus::ERR_REJECTED);              // the doubt reads robot
  CHECK(fake_sent_count() == 0);
  fake_advance(4'000'000);                                              // 6 s after the first frame
  stage_leg(0, ST_CL, 2.0f, 0.0f);
  CHECK(set_state(0, ST_IDLE) == RpcStatus::OK);
  CHECK(sdo_write(0) == RpcStatus::OK);
  CHECK(reboot(0) == RpcStatus::OK);
  CHECK(reboot(AXIS_ALL) == RpcStatus::OK);
  CHECK(fake_sent_count() == 3u + (size_t)CanBridge::NUM_AXES);
  // A Platform Teensy frame latches ROBOT (sticky): the same Leg is refused again.
  CanBridge::rig_note_platform_frame();
  fake_clear_sent();
  CHECK(set_state(0, ST_IDLE) == RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);
}
