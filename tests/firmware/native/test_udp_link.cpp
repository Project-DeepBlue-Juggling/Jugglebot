// =============================================================================
//  test_udp_link.cpp — compiled test of udp_link.cpp's NetLock coverage + RX
//  drain budget (NetLock-coverage hardening)
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

TEST_CASE("NetLock covers the lwIP pump AND every socket TX/RX") {
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

TEST_CASE("both sockets' receive queues are deepened (RPC FW 22, stream FW 40)") {
  // QNEthernet's default queue is ONE packet and a full queue overwrites the
  // oldest, so only the last RPC request of a 1 ms task_net tick survived. The
  // RPC socket queues RPC_RX_QUEUE_CAPACITY (<= the drain budget, so no added
  // latency — static_assert in udp_link.cpp). FW 40 (FEAT-41) gives the stream
  // socket the same depth: it was left at the default until then, and a host
  // heartbeat landing in the same tick as a setpoint overwrote the setpoint.
  netlockprobe::reset();
  CanBridge::udp_link_init();
  EthernetUDP* rpc = _registry()[JbUdp::PORT_RPC];
  EthernetUDP* stream = _registry()[JbUdp::PORT_STREAM];
  REQUIRE(rpc != nullptr);
  REQUIRE(stream != nullptr);
  CHECK(rpc->receive_queue_capacity() == CanBridge::RPC_RX_QUEUE_CAPACITY);
  CHECK(CanBridge::RPC_RX_QUEUE_CAPACITY >= 4);   // the fw-update pipeline depth
  CHECK(CanBridge::RPC_RX_QUEUE_CAPACITY <= CanBridge::UDP_RX_DRAIN_BUDGET);
  CHECK(stream->receive_queue_capacity() == CanBridge::STREAM_RX_QUEUE_CAPACITY);
  CHECK(CanBridge::STREAM_RX_QUEUE_CAPACITY >= 2);   // a setpoint AND a heartbeat in one tick
  CHECK(CanBridge::STREAM_RX_QUEUE_CAPACITY <= CanBridge::UDP_RX_DRAIN_BUDGET);
}

// Encode a valid HEARTBEAT_J2T frame (no handler registered → counted, dropped).
static uint16_t make_heartbeat(uint8_t* out, uint16_t out_cap, uint16_t seq) {
  const uint8_t payload[JbUdp::HEARTBEAT_J2T_SIZE] = {0};
  return JbUdp::encode_frame(JbUdp::MsgType::HEARTBEAT_J2T, seq,
                             payload, sizeof(payload), out, out_cap);
}

TEST_CASE("FW 40: a setpoint and a heartbeat landing in ONE task_net tick both reach the dispatcher") {
  // The FEAT-41 defect, at the socket. The host's 40 Hz SETPOINT and 10 Hz
  // HEARTBEAT_J2T share the stream socket and one sequence counter. With the
  // library's one-packet queue the heartbeat (arriving second, same 1 ms tick)
  // overwrote the setpoint: rx_setpoints stayed put, seq_gaps and the queue's
  // dropped count each rose by one. With the FW 40 queue both are drained, in
  // order, in that tick, and all three loss witnesses stay flat.
  uint8_t sp[64], hb[64];
  auto run_tick = [&](size_t capacity, uint16_t seq0) {
    netlockprobe::reset();
    CanBridge::udp_link_init();
    EthernetUDP* stream = _registry()[JbUdp::PORT_STREAM];
    REQUIRE(stream != nullptr);
    { CanBridge::NetLock lk; stream->setReceiveQueueCapacity(capacity); }
    stream->clear();
    // Prime the sequence tracker with one in-order setpoint, alone in its tick.
    const uint16_t n0 = make_setpoint(sp, sizeof(sp), seq0);
    stream->deliver(sp, n0);
    CanBridge::udp_link_service();
    const CanBridge::UdpStats before = CanBridge::udp_link_stats();
    const uint32_t dropped0 = CanBridge::udp_link_stream_queue_dropped();
    // The tick under test: setpoint, then heartbeat, before the drain runs.
    const uint16_t n1 = make_setpoint(sp, sizeof(sp), (uint16_t)(seq0 + 1));
    const uint16_t n2 = make_heartbeat(hb, sizeof(hb), (uint16_t)(seq0 + 2));
    stream->deliver(sp, n1);
    stream->deliver(hb, n2);
    CanBridge::udp_link_service();
    const CanBridge::UdpStats after = CanBridge::udp_link_stats();
    struct R { uint32_t setpoints, frames, gaps, dropped; } r{
        after.rx_setpoints - before.rx_setpoints, after.rx_frames - before.rx_frames,
        after.seq_gaps - before.seq_gaps,
        CanBridge::udp_link_stream_queue_dropped() - dropped0};
    return r;
  };
  SUBCASE("library default of ONE packet (FW <= 39): the setpoint is overwritten") {
    const auto r = run_tick(1, 100);
    CHECK(r.setpoints == 0u);   // the setpoint never reached the dispatcher
    CHECK(r.frames == 1u);      // only the heartbeat did
    CHECK(r.gaps == 1u);        // the hole is visible in the shared sequence
    CHECK(r.dropped == 1u);     // and in the library's own count
  }
  SUBCASE("FW 40 queue: both delivered, nothing counted as lost") {
    const auto r = run_tick(CanBridge::STREAM_RX_QUEUE_CAPACITY, 200);
    CHECK(r.setpoints == 1u);
    CHECK(r.frames == 2u);
    CHECK(r.gaps == 0u);
    CHECK(r.dropped == 0u);
  }
  CHECK(netlockprobe::violations() == 0);
}
