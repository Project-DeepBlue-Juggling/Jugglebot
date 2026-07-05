// =============================================================================
//  version_check.cpp — Phase 3 firmware Get_Version sweep + raw-version cache
// =============================================================================
//  See version_check.h for the design + the firmware/Jetson split. This TU owns
//  the per-axis raw-version cache and the bus-paced sweep; it parses NO versions.
// =============================================================================
#include "version_check.h"

#include "axis_state.h"        // axes[], NUM_AXES, HAND_AXIS, heartbeat_seen
#include "can_buses.h"         // can_jugglebot_send, jugglebot_commands_allowed
#include "odrive_protocol.h"   // ODrive::encode_get_version
#include "udp_protocol.h"      // JbUdp::RpcArgs::ResultAxisVersions
#include "time_base.h"         // micros64 (re-query pacing, item 20)

#include <cstring>

namespace CanBridge {

// Per-axis raw Get_Version payload (8 bytes each) + bitmasks. NUM_AXES = 7 (legs
// 0..5 + hand at HAND_AXIS=6) — the Jugglebot axes the bridge owns on CAN3.
//   s_received_mask   bit i ⇒ axis i's Get_Version reply has been cached
//   s_query_sent_mask bit i ⇒ a Get_Version has been sent to axis i this boot
// Written by version_record() (CAN3 RX context) + version_check_step()
// (cold-start monitor task), read by version_fill_blob() (net/RPC context).
// Single-byte volatile mask accesses are atomic on Cortex-M7; the received bit is
// published AFTER the 8 bytes (a compiler barrier), so a reader seeing a set bit
// always sees valid bytes — no seqlock (cf. axis_state.h note).
static volatile uint8_t s_version_raw[NUM_AXES][8] = {{0}};
static volatile uint8_t s_received_mask   = 0;
static volatile uint8_t s_query_sent_mask = 0;
static uint64_t         s_last_query_us   = 0;   // last (re)query send — re-query pacing (item 20)

void version_check_init() {
  // Re-arm: clear the masks + the raw cache (file-statics already boot zeroed; this
  // is the test-isolation seam and a future on-target re-arm hook). Production
  // correctness needs only the masks (the received mask gates every valid read), but
  // zeroing the raw bytes too makes a re-arm present a clean cache and keeps the
  // native tests order-independent (a version_record() in one case would otherwise
  // leave stale raw bytes visible to a later case's fill_blob).
  s_received_mask   = 0;
  s_query_sent_mask = 0;
  s_last_query_us   = 0;   // re-query pacing (item 20)
  for (uint8_t i = 0; i < NUM_AXES; ++i)
    for (uint8_t j = 0; j < 8; ++j) s_version_raw[i][j] = 0;
}

void version_record(uint8_t axis, const uint8_t* d8) {
  if (axis >= NUM_AXES || d8 == nullptr) return;
  for (uint8_t i = 0; i < 8; ++i) s_version_raw[axis][i] = d8[i];
  asm volatile("" ::: "memory");                 // publish bytes before the bit
  s_received_mask |= (uint8_t)(1u << axis);
}

// Re-query pacing (item 20). The first-pass sweep queries each present axis once
// and sets s_query_sent_mask; if that reply is lost (a bus glitch or an ODrive too
// busy to answer), the axis's received bit never sets and its version stays
// permanently missing — the Jetson's validate_group then reports a spurious
// version mismatch for a live, healthy axis. So after the first pass, re-query one
// present-but-UNRECEIVED axis per this interval (still one frame per tick) until
// every present axis has replied, then idle. 1 s keeps a never-replying axis to a
// slow trickle rather than a per-tick flood at the 100 Hz task rate.
static constexpr uint64_t VERSION_REQUERY_INTERVAL_US = 1000000ull;   // 1 s

void version_check_step() {
  // can_node parity (one-per-tick sweep, _send_next_version_query): send ONE
  // Get_Version to the next PRESENT-but-unqueried Jugglebot axis. "Present" =
  // ever heartbeated (axes[i].heartbeat_seen — the present-axis predicate), so a
  // partial bench rig queries only present axes and the full robot queries all 7.
  // Gated on jugglebot_commands_allowed() so a confirmed-dead CAN3 is never
  // pushed (an un-ACKed TX climbs the FlexCAN TEC). At most one frame per tick →
  // bus-paced; idle once every present axis has REPLIED (the steady state).
  if (!jugglebot_commands_allowed()) return;
  const uint64_t now = micros64();
  // First pass: send to the next present-but-unqueried axis (fast, as before).
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    const uint8_t bit = (uint8_t)(1u << i);
    if ((s_query_sent_mask & bit) || !axes[i].heartbeat_seen) continue;
    if (can_jugglebot_send(ODrive::encode_get_version(i))) {
      s_query_sent_mask |= bit;
      s_last_query_us = now;
    }
    return;   // one Get_Version per tick (bus pacing)
  }
  // Re-query pass: every present axis has been queried at least once. If any is
  // still unreceived and the re-query interval has elapsed since the last send,
  // re-send ONE — a single lost reply must not leave the version permanently
  // missing. Keys on s_received_mask (not s_query_sent_mask) so a replied axis is
  // never re-queried; paced by s_last_query_us so it never floods the bus.
  if (now - s_last_query_us < VERSION_REQUERY_INTERVAL_US) return;
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    const uint8_t bit = (uint8_t)(1u << i);
    if ((s_received_mask & bit) || !axes[i].heartbeat_seen) continue;
    if (can_jugglebot_send(ODrive::encode_get_version(i)))
      s_last_query_us = now;
    return;   // one re-query per tick (bus pacing)
  }
}

uint16_t version_fill_blob(uint8_t* out, uint16_t cap) {
  using JbUdp::RpcArgs::ResultAxisVersions;
  static_assert(sizeof(ResultAxisVersions) == 1 + NUM_AXES * 8,
                "ResultAxisVersions layout drift vs NUM_AXES*8 raw bytes");
  if (out == nullptr || cap < sizeof(ResultAxisVersions)) return 0;
  ResultAxisVersions r{};
  r.received_mask = s_received_mask;
  for (uint8_t i = 0; i < NUM_AXES; ++i)
    for (uint8_t j = 0; j < 8; ++j)
      r.raw[i * 8 + j] = s_version_raw[i][j];
  memcpy(out, &r, sizeof(r));
  return (uint16_t)sizeof(r);
}

uint8_t version_received_mask()   { return s_received_mask; }
uint8_t version_query_sent_mask() { return s_query_sent_mask; }

}  // namespace CanBridge
