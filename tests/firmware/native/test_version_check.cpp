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
//    * version_fill_blob respects the result-buffer cap.
//
//  SCOPE: validates DECISION LOGIC, not FreeRTOS/ISR concurrency or 500 Hz
//  timing (see README.md). The firmware parses NO versions — the blob is decoded
//  + validated on the Jetson (tests/ros + tests/teensy_link).
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "udp_protocol.h"
#include "odrive_protocol.h"
#include "axis_state.h"
#include "fake_hal.h"

#include "version_check.cpp"   // the unit under test

using namespace CanBridge;

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
