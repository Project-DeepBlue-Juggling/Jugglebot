// =============================================================================
//  test_version_check.cpp — compiled test of the REAL version handshake
// =============================================================================
//  Drives the actual compiled version_check.cpp (it #includes the .cpp) against
//  the recording fake HAL, asserting the safety/parity-relevant BEHAVIOURS of the
//  Get_Version sweep + raw-version cache:
//
//    * the sweep sends exactly ONE Get_Version per present-but-unqueried
//      Jugglebot axis, ONE frame per tick (bus pacing — can_node parity) on the
//      right arbitration id (cmd 0x00, empty payload), and idles once swept;
//    * an ABSENT axis (no heartbeat) is never queried;
//    * the sweep is GATED on jugglebot_commands_allowed() (never push a frame
//      onto a confirmed-dead CAN3 — the un-ACKed-TX/TEC failure mode);
//    * version_record() caches the raw 8 bytes + sets the received bit, and
//      version_fill_blob() returns the ResultAxisVersions blob (mask + axis-major
//      raw) the bridge pulls — exercised through the GROWABLE inbound-CAN3
//      injection hook (fake_inject_can3_rx → fake_can3_rx_pop), the same
//      len>=8 + cmd==get_version routing can_buses.cpp decode_into_cache uses;
//    * version_fill_blob respects the result-buffer cap;
//    * the BALL BUTLER twin (can-bridge FW 20+) does the same on CAN1 for axes
//      7-8 — one frame per tick on its OWN bus, absent axes never queried, the
//      partner-presence gate honoured, absolute node ids converted to the
//      BB-relative cache/mask, and out-of-range ids rejected rather than
//      wrapping into another axis's slot.
//
//  SCOPE: validates DECISION LOGIC, not FreeRTOS/ISR concurrency or 500 Hz
//  timing (see README.md). The firmware parses NO versions — the blob is decoded
//  + validated on the Jetson (tests/ros + tests/teensy_link).
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>
#include <vector>

#include "udp_protocol.h"
#include "odrive_protocol.h"
#include "axis_state.h"
#include "fake_hal.h"

#include "version_check.cpp"   // the unit under test

using namespace CanBridge;

// ── Local recording CAN1 TX ──────────────────────────────────────────────────
//  can_bb_tx is stubbed HERE rather than in fake_hal.cpp because
//  test_rpc_dispatch.cpp already defines its own (it exercises the BB relay
//  methods), and a second definition in the shared HAL is a multiple-definition
//  link error for every binary that links both.
//
//  A SEPARATE recording list from fake_hal's CAN3 one, mirroring the firmware's
//  separate buses: "the BB sweep sent one frame" must not be satisfiable by a
//  Jugglebot frame, or the independence case below proves nothing.
//
//  In the firmware can_bb_tx carries its OWN partner-presence gate
//  (partner_recent on the CAN1 RX stamp) instead of going through
//  jugglebot_commands_allowed(); g_bb_partner_present models exactly that, so a
//  test cannot pass by exercising the wrong gate.
namespace CanBridge {
static std::vector<SentFrame> g_bb_sent;
static bool g_bb_partner_present = true;

TxResult can_bb_tx(const ODrive::CanFrame& f, uint8_t cls) {   // HAL: can_buses.h
  if (!g_bb_partner_present) return TxResult::FAILED;   // partner_recent refusing
  SentFrame s;
  s.id = f.id; s.len = f.len; s.cls = cls;
  for (int i = 0; i < 8; ++i) s.buf[i] = f.buf[i];
  g_bb_sent.push_back(s);
  return TxResult::MAILBOX;
}
}  // namespace CanBridge

static void bb_fake_reset() { g_bb_sent.clear(); g_bb_partner_present = true; }
static void fake_set_bb_partner_present(bool p) { g_bb_partner_present = p; }
static size_t fake_bb_sent_count() { return g_bb_sent.size(); }
static const SentFrame& fake_bb_sent_at(size_t i) { return g_bb_sent.at(i); }

// Clear every axis's latched present bit so each case starts from a known set.
static void clear_all_present() {
  for (uint8_t i = 0; i < NUM_AXES; ++i) axes[i].heartbeat_seen = false;
}

// Route an injected inbound CAN3 frame exactly as can_buses.cpp decode_into_cache
// does for the get_version case (DLC<8 dropped; cmd==get_version → version_record).
static void drain_can3_into_version_cache() {
  uint32_t id; uint8_t buf[8]; uint8_t len;
  while (fake_can3_rx_pop(id, buf, len)) {
    if (len < 8) continue;
    if (ODrive::cmd_of(id) == ODriveCmd::get_version)
      version_record(ODrive::axis_of(id), buf);
  }
}

TEST_CASE("sweep sends one Get_Version per present axis, one per tick, bus-paced") {
  fake_reset();
  clear_all_present();
  version_check_init();
  fake_set_commands_allowed(true);

  // Two present Jugglebot axes (a leg + the hand); the rest absent.
  axes[0].heartbeat_seen = true;
  axes[HAND_AXIS].heartbeat_seen = true;

  // Tick 1: exactly one Get_Version frame on the bus.
  version_check_step();
  REQUIRE(fake_sent_count() == 1);
  CHECK(ODrive::cmd_of(fake_sent_at(0).id) == ODriveCmd::get_version);
  CHECK(fake_sent_at(0).len == 0);   // Get_Version request carries no payload

  // Tick 2: the second present axis is queried (still one frame/tick).
  version_check_step();
  REQUIRE(fake_sent_count() == 2);
  CHECK(ODrive::cmd_of(fake_sent_at(1).id) == ODriveCmd::get_version);

  // The two frames targeted exactly the two present axes (0 and HAND_AXIS).
  const uint8_t a0 = ODrive::axis_of(fake_sent_at(0).id);
  const uint8_t a1 = ODrive::axis_of(fake_sent_at(1).id);
  CHECK(((a0 == 0 && a1 == HAND_AXIS) || (a0 == HAND_AXIS && a1 == 0)));
  CHECK((version_query_sent_mask() & (1u << 0)) != 0);
  CHECK((version_query_sent_mask() & (1u << HAND_AXIS)) != 0);

  // Tick 3+: no present-but-unqueried axes left → no more frames (idle).
  version_check_step();
  version_check_step();
  CHECK(fake_sent_count() == 2);
}

TEST_CASE("sweep re-queries a present axis whose Get_Version reply was lost") {
  fake_reset();          // clock → 0
  clear_all_present();
  version_check_init();
  fake_set_commands_allowed(true);

  axes[0].heartbeat_seen = true;   // one present axis; its reply will be "lost"

  // First pass: axis 0 queried once, then idle (query_sent set, received NOT set).
  version_check_step();
  REQUIRE(fake_sent_count() == 1);
  CHECK((version_query_sent_mask() & (1u << 0)) != 0);
  CHECK((version_received_mask()   & (1u << 0)) == 0);

  // Before the re-query interval elapses: no re-query.
  version_check_step();
  version_check_step();
  CHECK(fake_sent_count() == 1);

  // After the 1 s re-query interval: the unreceived axis is re-queried (bus-paced).
  fake_advance(1000001);
  version_check_step();
  REQUIRE(fake_sent_count() == 2);
  CHECK(ODrive::axis_of(fake_sent_at(1).id) == 0);
  CHECK(ODrive::cmd_of(fake_sent_at(1).id) == ODriveCmd::get_version);

  // The reply finally arrives → received bit set → no further re-queries even past
  // another interval (idle once every present axis has replied).
  const uint8_t v0[8] = {0x00, 0x03, 0x06, 0x00, 0x00, 0x06, 0x0B, 0x00};
  version_record(0, v0);
  fake_advance(2000000);
  version_check_step();
  version_check_step();
  CHECK(fake_sent_count() == 2);
}

TEST_CASE("sweep never queries an absent axis") {
  fake_reset();
  clear_all_present();
  version_check_init();
  fake_set_commands_allowed(true);

  axes[3].heartbeat_seen = true;   // only axis 3 present

  for (int t = 0; t < 10; ++t) version_check_step();
  REQUIRE(fake_sent_count() == 1);
  CHECK(ODrive::axis_of(fake_sent_at(0).id) == 3);
  CHECK(version_query_sent_mask() == (uint8_t)(1u << 3));
}

TEST_CASE("sweep is gated on jugglebot_commands_allowed (never command a dead bus)") {
  fake_reset();
  clear_all_present();
  version_check_init();
  fake_set_commands_allowed(false);   // CAN3 confirmed down

  for (uint8_t i = 0; i < NUM_AXES; ++i) axes[i].heartbeat_seen = true;

  for (int t = 0; t < 5; ++t) version_check_step();
  CHECK(fake_sent_count() == 0);              // nothing pushed onto the dead bus
  CHECK(version_query_sent_mask() == 0);      // and nothing latched as sent

  // Once the bus recovers, the sweep resumes.
  fake_set_commands_allowed(true);
  version_check_step();
  CHECK(fake_sent_count() == 1);
}

TEST_CASE("version_record caches raw bytes (via the inbound-CAN3 hook) and fill_blob returns mask+raw") {
  fake_reset();
  clear_all_present();
  version_check_init();

  // A realistic ODrive Get_Version reply for axis 2: proto, hw(p,v,var), fw(maj,min,rev), unreleased.
  const uint8_t v2[8] = {0x00, 0x03, 0x06, 0x00, 0x00, 0x06, 0x0B, 0x00};
  fake_inject_can3_rx(ODrive::arb_id(2, ODriveCmd::get_version), v2, 8);
  // A short frame for axis 1 must be dropped (mirrors the < 8 guard).
  const uint8_t shortf[4] = {1, 2, 3, 4};
  fake_inject_can3_rx(ODrive::arb_id(1, ODriveCmd::get_version), shortf, 4);
  drain_can3_into_version_cache();

  CHECK((version_received_mask() & (1u << 2)) != 0);
  CHECK((version_received_mask() & (1u << 1)) == 0);   // short frame dropped

  uint8_t blob[64];
  const uint16_t n = version_fill_blob(blob, sizeof(blob));
  REQUIRE(n == sizeof(JbUdp::RpcArgs::ResultAxisVersions));   // 57 = 1 + 7*8
  JbUdp::RpcArgs::ResultAxisVersions r{};
  memcpy(&r, blob, sizeof(r));
  CHECK(r.received_mask == (uint8_t)(1u << 2));
  for (uint8_t j = 0; j < 8; ++j) CHECK(r.raw[2 * 8 + j] == v2[j]);
  // An unreceived axis's slot stays zero.
  for (uint8_t j = 0; j < 8; ++j) CHECK(r.raw[0 * 8 + j] == 0);
}

TEST_CASE("version_fill_blob refuses a too-small buffer") {
  version_check_init();
  uint8_t tiny[16];
  CHECK(version_fill_blob(tiny, sizeof(tiny)) == 0);
  CHECK(version_fill_blob(nullptr, 64) == 0);
}


// ═══════════════════════════════════════════════════════════════════════════
//  Ball Butler sweep (CAN1 axes 7-8) — can-bridge FW 20
// ═══════════════════════════════════════════════════════════════════════════
//  Restores the half of can_node's BOOT firmware check that commit 5875531
//  dropped. Same decision logic as the Jugglebot sweep above, on a different
//  bus with a different presence gate, so it gets its own cases rather than
//  being assumed to inherit the ones above.

static void clear_all_bb_present() {
  for (uint8_t i = 0; i < NUM_BB_AXES; ++i) bb_axes[i].heartbeat_seen = false;
}

TEST_CASE("bb sweep sends one Get_Version per present BB axis, one per tick") {
  fake_reset();
  bb_fake_reset();
  clear_all_bb_present();
  bb_version_check_init();

  bb_axes[0].heartbeat_seen = true;   // node 7 (BB pitch)
  bb_axes[1].heartbeat_seen = true;   // node 8 (BB hand)

  bb_version_check_step();
  REQUIRE(fake_bb_sent_count() == 1);
  CHECK(ODrive::cmd_of(fake_bb_sent_at(0).id) == ODriveCmd::get_version);
  CHECK(fake_bb_sent_at(0).len == 0);

  bb_version_check_step();
  REQUIRE(fake_bb_sent_count() == 2);

  // Absolute node ids off the wire — NOT the BB-relative cache indices.
  const uint8_t n0 = ODrive::axis_of(fake_bb_sent_at(0).id);
  const uint8_t n1 = ODrive::axis_of(fake_bb_sent_at(1).id);
  CHECK(((n0 == BB_FIRST_NODE && n1 == BB_FIRST_NODE + 1) ||
         (n0 == BB_FIRST_NODE + 1 && n1 == BB_FIRST_NODE)));

  // Swept: idle, and NOTHING was put on the Jugglebot bus.
  bb_version_check_step();
  CHECK(fake_bb_sent_count() == 2);
  CHECK(fake_sent_count() == 0);
}

TEST_CASE("bb sweep never queries an absent axis") {
  fake_reset();
  bb_fake_reset();
  clear_all_bb_present();
  bb_version_check_init();

  bb_axes[1].heartbeat_seen = true;   // only node 8 present

  for (int i = 0; i < 5; ++i) bb_version_check_step();
  REQUIRE(fake_bb_sent_count() == 1);
  CHECK(ODrive::axis_of(fake_bb_sent_at(0).id) == BB_FIRST_NODE + 1);
}

TEST_CASE("bb sweep is gated on the CAN1 partner-presence check") {
  // A Ball Butler that is switched off must not be pushed frames: an un-ACKed
  // TX climbs the FlexCAN TEC. can_bb_tx refuses, and the sweep must not mark
  // the axis queried on a refusal — or a BB powered up later never resolves.
  fake_reset();
  bb_fake_reset();
  clear_all_bb_present();
  bb_version_check_init();
  bb_axes[0].heartbeat_seen = true;

  fake_set_bb_partner_present(false);
  for (int i = 0; i < 3; ++i) bb_version_check_step();
  CHECK(fake_bb_sent_count() == 0);
  CHECK(bb_version_query_sent_mask() == 0);

  // Partner returns → the sweep proceeds.
  fake_set_bb_partner_present(true);
  bb_version_check_step();
  CHECK(fake_bb_sent_count() == 1);
}

TEST_CASE("bb_version_record caches by ABSOLUTE node id into the relative slot") {
  fake_reset();
  bb_fake_reset();
  bb_version_check_init();

  uint8_t v7[8]; for (int i = 0; i < 8; ++i) v7[i] = (uint8_t)(0x10 + i);
  uint8_t v8[8]; for (int i = 0; i < 8; ++i) v8[i] = (uint8_t)(0x20 + i);

  bb_version_record(BB_FIRST_NODE, v7);
  CHECK(bb_version_received_mask() == 0b01);      // bit 0 ⇒ the FIRST BB axis
  bb_version_record(BB_FIRST_NODE + 1, v8);
  CHECK(bb_version_received_mask() == 0b11);

  uint8_t blob[64];
  const uint16_t n = bb_version_fill_blob(blob, sizeof(blob));
  REQUIRE(n == sizeof(JbUdp::RpcArgs::ResultBbAxisVersions));
  CHECK(blob[0] == 0b11);                          // received_mask
  CHECK(memcmp(blob + 1 + 0 * 8, v7, 8) == 0);     // axis-major from BB_FIRST_NODE
  CHECK(memcmp(blob + 1 + 1 * 8, v8, 8) == 0);
}

TEST_CASE("bb_version_record rejects ids outside the BB range") {
  // A stray frame from an unexpected node id must not land in a BB slot — the
  // reason the record takes the ABSOLUTE id and RANGE-CHECKS it rather than
  // trusting a pre-subtracted index.
  //
  // Asserted two ways, because the mask alone is too weak: the realistic wrong
  // implementation is not "no check at all" but an ALIASING one (a modulo, or a
  // missing BB_FIRST_NODE subtraction), which would quietly overwrite a slot
  // that already holds a real version. So the cache is SEEDED first and the
  // seeds must survive — a corrupted good slot is the failure that would
  // actually reach a bench, reporting one drive's firmware for another's.
  fake_reset();
  bb_fake_reset();
  bb_version_check_init();

  uint8_t good7[8], good8[8];
  for (int i = 0; i < 8; ++i) { good7[i] = (uint8_t)(0x10 + i); good8[i] = (uint8_t)(0x20 + i); }
  bb_version_record(BB_FIRST_NODE, good7);
  bb_version_record(BB_FIRST_NODE + 1, good8);
  REQUIRE(bb_version_received_mask() == 0b11);

  uint8_t bad[8]; for (int i = 0; i < 8; ++i) bad[i] = 0xAB;
  bb_version_record(0, bad);                        // leg 0
  bb_version_record(BB_FIRST_NODE - 1, bad);        // the hand — adjacent, not BB
  bb_version_record(BB_FIRST_NODE + NUM_BB_AXES, bad);
  bb_version_record(255, bad);
  bb_version_record(BB_FIRST_NODE, nullptr);        // null payload

  // Neither seeded slot was disturbed, and no extra bit was set.
  CHECK(bb_version_received_mask() == 0b11);
  uint8_t blob[64];
  REQUIRE(bb_version_fill_blob(blob, sizeof(blob)) ==
          sizeof(JbUdp::RpcArgs::ResultBbAxisVersions));
  CHECK(memcmp(blob + 1 + 0 * 8, good7, 8) == 0);
  CHECK(memcmp(blob + 1 + 1 * 8, good8, 8) == 0);
}

TEST_CASE("an out-of-range id on an EMPTY cache sets no received bit") {
  // The mask half of the contract, on a fresh cache: an unexpected node id must
  // not make an axis look like it answered.
  fake_reset();
  bb_fake_reset();
  bb_version_check_init();

  uint8_t bad[8]; for (int i = 0; i < 8; ++i) bad[i] = 0xAB;
  bb_version_record(0, bad);
  bb_version_record(BB_FIRST_NODE - 1, bad);
  bb_version_record(BB_FIRST_NODE + NUM_BB_AXES, bad);
  CHECK(bb_version_received_mask() == 0);
}

TEST_CASE("bb_version_fill_blob respects the result-buffer cap") {
  fake_reset();
  bb_fake_reset();
  bb_version_check_init();
  uint8_t small[4];
  CHECK(bb_version_fill_blob(small, sizeof(small)) == 0);
  CHECK(bb_version_fill_blob(nullptr, 64) == 0);
}

TEST_CASE("the two sweeps are independent") {
  // A dead Ball Butler must not stall the Jugglebot sweep, and vice versa —
  // the reason these are separate state rather than one widened array.
  fake_reset();
  bb_fake_reset();
  clear_all_present();
  clear_all_bb_present();
  version_check_init();
  bb_version_check_init();
  fake_set_commands_allowed(true);
  fake_set_bb_partner_present(false);   // BB switched off

  axes[0].heartbeat_seen = true;
  bb_axes[0].heartbeat_seen = true;

  version_check_step();
  bb_version_check_step();
  CHECK(fake_sent_count() == 1);        // Jugglebot swept regardless
  CHECK(fake_bb_sent_count() == 0);

  // And the converse: a down CAN3 gate must not stop the BB sweep.
  fake_reset();
  bb_fake_reset();
  clear_all_present();
  clear_all_bb_present();
  version_check_init();
  bb_version_check_init();
  fake_set_commands_allowed(false);
  axes[0].heartbeat_seen = true;
  bb_axes[0].heartbeat_seen = true;

  version_check_step();
  bb_version_check_step();
  CHECK(fake_sent_count() == 0);
  CHECK(fake_bb_sent_count() == 1);
}
