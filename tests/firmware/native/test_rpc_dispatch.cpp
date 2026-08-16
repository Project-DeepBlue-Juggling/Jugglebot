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
static bool     g_stow_pending = false;       // drives fault_stow_pending()
static uint8_t  g_homing_axis = 0xFF, g_activate_axis = 0xFF, g_deactivate_axis = 0xFF;
static uint16_t g_homing_ret = JbUdp::RpcStatus::OK;
static uint16_t g_activate_ret = JbUdp::RpcStatus::OK;
static uint16_t g_deactivate_ret = JbUdp::RpcStatus::OK;
static uint16_t g_hand_traj_ret = JbUdp::RpcStatus::OK;
static int      g_hand_traj_calls = 0;
static uint16_t g_version_len = 8;            // version_fill_blob returned length
static int      g_tilt_calls = 0, g_state_read_calls = 0, g_state_write_calls = 0;
// Clapboard downlink (CLAP_SEND). can_cone_send is stubbed here because fake_hal
// does NOT define it — clap_link is only its second caller ever — and rpc.cpp now
// links against the cone-bus gate. clap_tx_enqueue_burst is stubbed for ROUTING
// ISOLATION, exactly as HandOps/Relay are: the ring's own behaviour (all-or-
// nothing, FIFO, one-frame-per-tick drain) is driven by test_clap_link against
// the REAL clap_link.cpp.
static bool     g_cone_partner = true;        // drives cone_partner_present()
static int      g_cone_sent = 0;              // can_cone_send call count
static int      g_clap_bursts = 0;            // clap_tx_enqueue_burst call count
static uint8_t  g_clap_frames = 0;            // frames in the last accepted burst
static bool     g_clap_enqueue_ok = true;     // drives clap_tx_enqueue_burst's return

static void drv_reset() {
  g_bus_transmittable = true;
  g_bb_sent = 0; g_bb_last = ODrive::CanFrame{}; g_bb_send_ok = true;
  g_clear_calls = 0; g_reboot_calls = 0; g_stow_pending = false;
  g_homing_axis = g_activate_axis = g_deactivate_axis = 0xFF;
  g_homing_ret = g_activate_ret = g_deactivate_ret = JbUdp::RpcStatus::OK;
  g_hand_traj_ret = JbUdp::RpcStatus::OK; g_hand_traj_calls = 0;
  g_version_len = 8;
  g_tilt_calls = g_state_read_calls = g_state_write_calls = 0;
  g_cone_partner = true; g_cone_sent = 0;
  g_clap_bursts = 0; g_clap_frames = 0; g_clap_enqueue_ok = true;
}

// ── can_buses.h ──
bool jugglebot_bus_transmittable() { return g_bus_transmittable; }
bool can_bb_send(const ODrive::CanFrame& f) { g_bb_sent++; g_bb_last = f; return g_bb_send_ok; }
bool can_cone_send(const ODrive::CanFrame&) { g_cone_sent++; return true; }
bool cone_partner_present() { return g_cone_partner; }

// ── clap_link.h ──
bool clap_tx_enqueue_burst(const uint32_t*, const uint8_t*, const uint8_t*, uint8_t count) {
  g_clap_bursts++;
  if (!g_clap_enqueue_ok) return false;
  g_clap_frames = count;
  return true;
}

// ── fault_machine.h ──
void fault_notify_clear_errors()  { g_clear_calls++; }
void fault_notify_reboot_started() { g_reboot_calls++; }
bool fault_stow_pending()          { return g_stow_pending; }

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
  ArgSdoRead sd{}; sd.axis = HAND_AXIS; sd.endpoint = 726;
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

// ── CLAP_SEND: the clapboard downlink relay ───────────────────────────────────
// THE SAFETY-RELEVANT BEHAVIOUR is the first case: a closed TX presence gate must
// return ERR_BUS_DOWN having enqueued ZERO frames.  An un-ACKed TX on a
// partner-less bus retransmits forever, pinning TEC at the error-passive
// threshold and escalating to bus-off across supply ramps — on a bus whose analog
// drive path is already known-degraded.  The gate is checked once, up front,
// because the drain runs long after this RPC has returned.

TEST_CASE("CLAP_SEND with the TX gate closed → ERR_BUS_DOWN and ZERO frames enqueued") {
  reset_all();
  g_cone_partner = false;                     // no partner seen within the window
  JbUdp::RpcArgs::ArgClapSend a{};
  a.count = 3;
  for (uint8_t i = 0; i < 3; ++i) { a.can_id[i] = 0x7E8u; a.len[i] = 8; }
  CHECK(call(RpcMethod::CLAP_SEND, a) == RpcStatus::ERR_BUS_DOWN);
  CHECK(g_clap_bursts == 0);                  // nothing even offered to the ring
  CHECK(g_cone_sent == 0);                    // and nothing on the wire
}

TEST_CASE("CLAP_SEND with the gate open enqueues the whole burst exactly once") {
  reset_all();
  JbUdp::RpcArgs::ArgClapSend a{};
  a.count = 41;                               // the worst-case slate transaction
  for (uint8_t i = 0; i < 41; ++i) { a.can_id[i] = 0x7E8u; a.len[i] = 8; }
  a.can_id[40] = 0x7E9u;                      // ...with the commit last
  CHECK(call(RpcMethod::CLAP_SEND, a) == RpcStatus::OK);
  CHECK(g_clap_bursts == 1);
  CHECK(g_clap_frames == 41);
  // Nothing is sent synchronously: the drain is paced one frame per tick and runs
  // on task_time_sync, long after this RPC has returned.
  CHECK(g_cone_sent == 0);
}

TEST_CASE("CLAP_SEND arg validation happens BEFORE the ring is touched") {
  reset_all();
  JbUdp::RpcArgs::ArgClapSend a{};

  a.count = 0;                                // an empty burst is not a transaction
  CHECK(call(RpcMethod::CLAP_SEND, a) == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_clap_bursts == 0);

  reset_all();
  a.count = (uint8_t)(JbUdp::CLAP_MAX_FRAMES + 1);   // more slots than the struct has
  CHECK(call(RpcMethod::CLAP_SEND, a) == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_clap_bursts == 0);

  reset_all();
  // DLC bound: len is copied into CAN_message_t::len and > 8 is not a legal
  // classic-CAN frame.  This is framing/memory safety — the clapboard's stricter
  // "must be exactly 8" rule is deliberately NOT enforced in firmware, so DLC 8 is
  // accepted here and so is a short frame; only an over-long one is refused.
  a.count = 2; a.can_id[0] = 0x7E8u; a.len[0] = 8; a.can_id[1] = 0x7E8u; a.len[1] = 9;
  CHECK(call(RpcMethod::CLAP_SEND, a) == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_clap_bursts == 0);
}

TEST_CASE("a malformed CLAP_SEND reports ERR_BAD_ARGS even with the bus down") {
  // ORDER MATTERS, and only a closed gate can prove it: with the gate checked
  // first, every case below would answer ERR_BUS_DOWN and send an operator
  // hunting a wiring fault for a caller-side coding error.  Same ordering as
  // BB_THROW (range-check, then the presence gate).
  reset_all();
  g_cone_partner = false;
  JbUdp::RpcArgs::ArgClapSend a{};

  a.count = 0;
  CHECK(call(RpcMethod::CLAP_SEND, a) == RpcStatus::ERR_BAD_ARGS);

  a.count = (uint8_t)(JbUdp::CLAP_MAX_FRAMES + 1);
  CHECK(call(RpcMethod::CLAP_SEND, a) == RpcStatus::ERR_BAD_ARGS);

  a.count = 1; a.can_id[0] = 0x7E8u; a.len[0] = 9;
  CHECK(call(RpcMethod::CLAP_SEND, a) == RpcStatus::ERR_BAD_ARGS);

  // ...and a WELL-FORMED request on the same down bus still reports the bus.
  a.len[0] = 8;
  CHECK(call(RpcMethod::CLAP_SEND, a) == RpcStatus::ERR_BUS_DOWN);
  CHECK(g_clap_bursts == 0);
}

TEST_CASE("CLAP_SEND with a short arg blob → ERR_BAD_ARGS (take() bound)") {
  reset_all();
  uint8_t result[64]; uint16_t res_len = 0;
  uint8_t truncated[16] = {0};
  CHECK(Rpc::dispatch(RpcMethod::CLAP_SEND, truncated, sizeof(truncated), result, res_len)
        == RpcStatus::ERR_BAD_ARGS);
  CHECK(g_clap_bursts == 0);
}

TEST_CASE("a full ring surfaces as ERR_REJECTED, not ERR_BAD_ARGS") {
  // The request was well-formed; the bridge is simply still draining an earlier
  // transaction.  Enqueue is all-or-nothing, so nothing reached the wire — which
  // is strictly better than a fragment the clapboard would answer CRC_MISMATCH to.
  reset_all();
  g_clap_enqueue_ok = false;
  JbUdp::RpcArgs::ArgClapSend a{};
  a.count = 5;
  for (uint8_t i = 0; i < 5; ++i) { a.can_id[i] = 0x7E8u; a.len[i] = 8; }
  CHECK(call(RpcMethod::CLAP_SEND, a) == RpcStatus::ERR_REJECTED);
  CHECK(g_clap_bursts == 1);                  // offered...
  CHECK(g_cone_sent == 0);                    // ...and refused whole
}
