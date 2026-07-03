// =============================================================================
//  test_udp_link.cpp — compiled test of udp_link.cpp's NetLock coverage + RX
//  drain budget (Tier-2 hardening item 15)
// =============================================================================
//  Two hazards the fix closes, witnessed here as COMPILED assertions:
//
//   (1) LOCK COVERAGE. QNEthernet's lwIP is NO_SYS=1 (non-reentrant). Before the
//       fix, udp_link_service() pumped `Ethernet.loop()` OUTSIDE any NetLock while
//       prio-3 TX tasks could be mid-send_to() (holding NetLock) on the same pbuf
//       pool → corruption. The fix wraps the pump in its own NetLock scope. The
//       fake QNEthernet (hal_shims/QNEthernet.h) calls netlockprobe::require_held()
//       at the entry of Ethernet.loop() AND every socket method; the fake recursive
//       mutex (hal_shims/arduino_freertos.h) tracks the lock depth. Driving
//       udp_link_service()+send proves NOTHING runs unlocked (violations()==0).
//       Removing the pump's NetLock wrap trips require_held() on Ethernet.loop() →
//       violations()>0. This test is the regression witness.
//
//   (2) DRAIN BUDGET. drain_socket() was an unbounded for(;;); under a flood the
//       prio-4 net task could spin, starving the fault machine / homing / diag. The
//       fix caps it at UDP_RX_DRAIN_BUDGET(=8) frames per socket per wake and counts
//       a budget-bound (frame still queued) in UdpStats::drain_cap_hits, draining
//       the backlog on the next wake (not dropping it). Staging 20 packets proves
//       exactly 8 drain per wake, cap_hits increments, and the remainder survives.
//
//  SCOPE: this validates lock COVERAGE (which calls run under the lock) and the
//  drain budget — NOT true preemptive reentrancy, which stays an on-hardware-replay
//  gap (README.md). #includes udp_link.cpp to reach its file-statics; NOT linked.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "net_lock_probe.h"

// Pull the REAL udp_link.cpp into this TU so its file-static sockets/stats/drain
// loop are exercised directly (mirrors test_udp_framing including udp_protocol.h).
#include "udp_link.cpp"

// ── Host stubs for udp_link.cpp's two non-inline externals ───────────────────
namespace CanBridge {
// micros64: a monotone fake clock (its VALUE is irrelevant to these tests; only the
// atomic_write into s_last_rx_us is exercised). Defined here so we need not link
// time_base.o.
uint64_t micros64() { static uint64_t t = 0; t += 1000; return t; }
// net_ethernet_service() is a 1-line forwarder to the lwIP pump; the REAL one
// (net_ethernet.cpp) is exactly `Ethernet.loop();`. Reproduced faithfully so the
// fake pump's require_held() witnesses that udp_link_service() wraps it in NetLock.
// (Stubbed rather than linking net_ethernet.o, whose net_ethernet_begin would drag
// millis()/delay() definitions this test does not need.)
void net_ethernet_service() { qindesign::network::Ethernet.loop(); }
}  // namespace CanBridge

using qindesign::network::EthernetUDP;
using qindesign::network::_registry;

// Encode a valid SETPOINT frame (no handler is registered → it is counted in
// rx_frames and otherwise dropped, ideal for counting drained frames).
static uint16_t make_setpoint(uint8_t* out, uint16_t out_cap, uint16_t seq) {
  const uint8_t payload[6] = {0, 0, 0, 0, 0, 0};
  return JbUdp::encode_frame(JbUdp::MsgType::SETPOINT, seq,
                             payload, sizeof(payload), out, out_cap);
}

TEST_CASE("NetLock covers the lwIP pump AND every socket TX/RX (Tier-2 item 15)") {
  netlockprobe::reset();
  CanBridge::udp_link_init();               // binds s_stream/s_rpc under NetLock
  const unsigned loops0 = qindesign::network::Ethernet.loops_;

  // A full service (pump + drain both empty sockets) + a TX on each channel.
  CanBridge::udp_link_service();
  const uint8_t pl[4] = {0xDE, 0xAD, 0xBE, 0xEF};
  CHECK(CanBridge::udp_send_stream(JbUdp::MsgType::TELEMETRY, pl, sizeof(pl)));
  CHECK(CanBridge::udp_send_rpc(JbUdp::MsgType::RPC_RESPONSE, pl, sizeof(pl)));

  // The lwIP pump actually ran (so its require_held() had a chance to trip)...
  CHECK(qindesign::network::Ethernet.loops_ == loops0 + 1);
  // ...and NOT ONE fake QNEthernet call ran outside NetLock. Pre-fix (unlocked
  // pump) Ethernet.loop() alone makes this > 0 — the regression witness.
  CHECK(netlockprobe::violations() == 0);
}

TEST_CASE("RX drain is bounded by UDP_RX_DRAIN_BUDGET; the backlog is not lost") {
  netlockprobe::reset();
  CanBridge::udp_link_init();
  EthernetUDP* stream = _registry()[JbUdp::PORT_STREAM];
  REQUIRE(stream != nullptr);
  stream->clear();

  // Stage 20 pending frames on the stream socket.
  uint8_t frame[64];
  for (uint16_t i = 0; i < 20; ++i) {
    const uint16_t total = make_setpoint(frame, sizeof(frame), i);
    REQUIRE(total > 0);
    stream->stage(frame, total);
  }

  const uint32_t cap0 = CanBridge::udp_link_stats().drain_cap_hits;
  const uint32_t rx0 = CanBridge::udp_link_stats().rx_frames;

  // Wake 1: drains EXACTLY the budget (8), 12 remain, one cap hit recorded.
  CanBridge::udp_link_service();
  CHECK(stream->consumed() == 8);
  CHECK(stream->remaining() == 12);
  CHECK(CanBridge::udp_link_stats().rx_frames - rx0 == 8u);
  CHECK(CanBridge::udp_link_stats().drain_cap_hits - cap0 == 1u);

  // Wake 2: the NEXT 8 (not lost), 4 remain, a second cap hit.
  CanBridge::udp_link_service();
  CHECK(stream->consumed() == 16);
  CHECK(stream->remaining() == 4);
  CHECK(CanBridge::udp_link_stats().drain_cap_hits - cap0 == 2u);

  // Wake 3: the final 4 — a clean empty-exit within budget, NO further cap hit.
  CanBridge::udp_link_service();
  CHECK(stream->consumed() == 20);
  CHECK(stream->remaining() == 0);
  CHECK(CanBridge::udp_link_stats().drain_cap_hits - cap0 == 2u);
  CHECK(CanBridge::udp_link_stats().rx_frames - rx0 == 20u);

  CHECK(netlockprobe::violations() == 0);
}

TEST_CASE("drain_cap_hits stays 0 at <= 1 frame per service() call") {
  netlockprobe::reset();
  CanBridge::udp_link_init();
  EthernetUDP* stream = _registry()[JbUdp::PORT_STREAM];
  REQUIRE(stream != nullptr);
  stream->clear();

  const uint32_t cap0 = CanBridge::udp_link_stats().drain_cap_hits;
  uint8_t frame[64];
  for (uint16_t i = 0; i < 5; ++i) {
    const uint16_t total = make_setpoint(frame, sizeof(frame), i);
    REQUIRE(total > 0);
    stream->stage(frame, total);      // exactly one pending frame this wake
    CanBridge::udp_link_service();    // drains it, then empty-exits (n < budget)
    CHECK(stream->remaining() == 0);
  }
  CHECK(CanBridge::udp_link_stats().drain_cap_hits - cap0 == 0u);
  CHECK(netlockprobe::violations() == 0);
}
