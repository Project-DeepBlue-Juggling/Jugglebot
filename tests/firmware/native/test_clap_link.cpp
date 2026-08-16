// =============================================================================
//  test_clap_link.cpp — compiled test of the REAL clap_link.cpp CLAP_LINK beacon
// =============================================================================
//  Drives the actual compiled clap_link.cpp (it #includes the .cpp) against a
//  recording can_cone_send + the fake HAL's controllable clock, asserting the two
//  wire rules that are enforced ONLY by the peer repo's code and are silent on
//  both sides when broken:
//
//    * EVERY emitted frame is DLC 8 on id 0x7EA with bytes 1-7 zero.  Byte 0 is
//      the only meaningful byte, so the natural implementation is a 1-byte frame
//      — which the clapboard silently drops (its can_frames.h enforces a strict
//      length; docs/protocol.md never states the rule).  The panel would sit in
//      screensaver forever with no error anywhere.  This is the plan's named
//      most-likely-first-integration-bug, so it is pinned by a compiled test
//      rather than by a comment.
//    * EXACTLY 2 Hz.  The clapboard treats a 3 s gap as bridge-dead, so 2 Hz is
//      six missed frames of margin; the cone bus's analog drive path is
//      known-degraded (logbook 2026-07-31), so raising the rate is a real cost.
//
//  Plus the LinkState → byte-0 mapping, which is where a wrong answer is
//  dangerous in the safe direction only if it errs towards 0: screensaver is the
//  safe default, so anything short of a healthy bidirectional link must report
//  DOWN.
//
//  This TU is natively testable precisely BECAUSE link_state is a parameter —
//  .ino code is compiled by no test.  See clap_link.h.
//
//  SCOPE: decision logic + wire shape.  Not FreeRTOS scheduling and not the real
//  FlexCAN TX path (can_cone_send is stubbed; can_buses.cpp compiles in no native
//  binary — see native/README.md).
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "udp_protocol.h"
#include "protocol_config.h"
#include "odrive_protocol.h"
#include "can_buses.h"
#include "fake_hal.h"

// ── Driver-local stub: the cone-bus TX leaf clap_link.cpp calls ──────────────
//  fake_hal defines can_jugglebot_send but NOT can_cone_send (clap_link is its
//  first caller besides the time-sync fan-out), so it is recorded here.
namespace CanBridge {

static const int  CONE_CAP = 64;
static SentFrame  g_cone[CONE_CAP];
static int        g_cone_n = 0;
static bool       g_cone_send_ok = true;

static void cone_reset() { g_cone_n = 0; g_cone_send_ok = true; }

bool can_cone_send(const ODrive::CanFrame& f) {   // HAL: can_buses.h
  if (g_cone_n < CONE_CAP) {
    g_cone[g_cone_n].id = f.id;
    g_cone[g_cone_n].len = f.len;
    memcpy(g_cone[g_cone_n].buf, f.buf, 8);
    ++g_cone_n;
  }
  return g_cone_send_ok;
}

}  // namespace CanBridge

#include "clap_link.cpp"   // the unit under test (reaches its file-statics)

using namespace CanBridge;

// Reset the whole driver: fake clock, recorded frames, emitter cadence.
static void reset_all() {
  fake_reset();
  cone_reset();
  clap_link_reset();
}

// Run `ticks` calls of clap_link_step at the production 100 Hz task rate.
static void run_ticks(int ticks, uint8_t link_state) {
  for (int i = 0; i < ticks; ++i) {
    clap_link_step(link_state);
    fake_advance(1000000u / 100u);   // TIME_SYNC_RATE_HZ = 100
  }
}

TEST_CASE("every CLAP_LINK frame is id 0x7EA, DLC 8, bytes 1-7 zero") {
  reset_all();

  clap_link_step(JbUdp::LinkState::UP);
  REQUIRE(g_cone_n == 1);
  CHECK(g_cone[0].id == ClapboardCanId::LINK);   // 0x7EA
  // THE named first-integration-bug guard.  A 1-byte frame is silently dropped
  // by the clapboard, leaving the panel in screensaver with no error on either
  // side.  protocol.md §8.5 documents the layout but not this rule.
  CHECK(g_cone[0].len == 8);
  CHECK(g_cone[0].buf[0] == 1);
  for (int b = 1; b < 8; ++b) {
    CHECK(g_cone[0].buf[b] == 0);   // "reserved, must be 0" (§8.5)
  }
}

TEST_CASE("byte 0 is 1 only for a healthy bidirectional link") {
  // Screensaver is the SAFE default, so every state short of UP must report DOWN
  // — an ambiguous link must never render as a live scene slate.
  CHECK(clap_link_state_byte(JbUdp::LinkState::UP) == 1);
  CHECK(clap_link_state_byte(JbUdp::LinkState::INIT) == 0);
  CHECK(clap_link_state_byte(JbUdp::LinkState::DEGRADED) == 0);
  CHECK(clap_link_state_byte(JbUdp::LinkState::LOST) == 0);

  reset_all();
  clap_link_step(JbUdp::LinkState::LOST);
  REQUIRE(g_cone_n == 1);
  CHECK(g_cone[0].buf[0] == 0);
  CHECK(g_cone[0].len == 8);        // DLC 8 in the DOWN direction too
}

TEST_CASE("the beacon runs at exactly 2 Hz off a 100 Hz caller") {
  reset_all();

  // The first call emits immediately: a clapboard powered up beside a running
  // bridge should not wait half a second to learn the link is up.
  clap_link_step(JbUdp::LinkState::UP);
  CHECK(g_cone_n == 1);

  // 49 further ticks (490 ms) must emit nothing...
  for (int i = 0; i < 49; ++i) {
    fake_advance(10000u);
    clap_link_step(JbUdp::LinkState::UP);
  }
  CHECK(g_cone_n == 1);

  // ...and the 50th (exactly 500 ms after the first) must emit.
  fake_advance(10000u);
  clap_link_step(JbUdp::LinkState::UP);
  CHECK(g_cone_n == 2);
}

TEST_CASE("one second of 100 Hz ticks yields exactly two frames, all DLC 8") {
  reset_all();

  run_ticks(100, JbUdp::LinkState::UP);   // 1.00 s of task ticks
  // t=0 and t=500 ms.  Two, not three: the tick at t=1.00 s has not run yet.
  CHECK(g_cone_n == 2);
  for (int i = 0; i < g_cone_n; ++i) {
    CHECK(g_cone[i].id == ClapboardCanId::LINK);
    CHECK(g_cone[i].len == 8);
  }

  // A second full second adds exactly two more — the cadence does not drift or
  // double-fire, which is what keeps a degraded bus quiet.
  run_ticks(100, JbUdp::LinkState::UP);
  CHECK(g_cone_n == 4);
}

TEST_CASE("a link-state change reaches the wire on the next beacon, not sooner") {
  reset_all();

  clap_link_step(JbUdp::LinkState::UP);
  REQUIRE(g_cone_n == 1);
  CHECK(g_cone[0].buf[0] == 1);

  // Link drops 10 ms later.  DELIBERATELY no change-triggered emit: link_state()
  // can chatter at the JETSON_LINK_TIMEOUT_US boundary, and an emit-on-change
  // would turn that chatter into a burst on the one bus that must stay quiet.
  // The cost is bounded at 500 ms of stale panel state, well inside the
  // clapboard's own 3 s bridge-dead rule.
  fake_advance(10000u);
  clap_link_step(JbUdp::LinkState::LOST);
  CHECK(g_cone_n == 1);

  fake_advance(490000u);            // now exactly 500 ms since the first emit
  clap_link_step(JbUdp::LinkState::LOST);
  REQUIRE(g_cone_n == 2);
  CHECK(g_cone[1].buf[0] == 0);     // DOWN reached the wire
  CHECK(g_cone[1].len == 8);
}

TEST_CASE("a refused send does not stall or re-burst the cadence") {
  // can_cone_send returns false when the bus-partner presence gate is closed (no
  // clapboard yet) or when the mailbox write defers.  The beacon deliberately
  // discards that result — exactly as broadcast_0x7dd() discards its three — so a
  // partner-less bus must not accumulate a backlog that fires all at once the
  // moment the clapboard appears.
  reset_all();
  g_cone_send_ok = false;

  run_ticks(100, JbUdp::LinkState::UP);
  CHECK(g_cone_n == 2);             // attempted at 2 Hz, no more and no fewer

  g_cone_send_ok = true;
  run_ticks(100, JbUdp::LinkState::UP);
  CHECK(g_cone_n == 4);             // no catch-up burst
}

// ── clap_tx: the paced downlink burst queue ──────────────────────────────────
//  The REAL ring, unlike test_rpc_dispatch's routing stub.  What matters:
//    * ALL-OR-NOTHING enqueue — a partially-queued transaction reaches the
//      clapboard as a fragment and comes back CRC_MISMATCH/INCOMPLETE, which
//      reads as a panel fault rather than as a bridge refusal;
//    * FIFO order — chunks may arrive in any order but CLAP_COMMIT must arrive
//      AFTER its chunks, and one FIFO-drained burst is what guarantees that with
//      no ordering logic in the firmware;
//    * ONE FRAME PER TICK, never a burst, on a bus whose analog drive path is
//      known-degraded;
//    * a closed gate DISCARDS rather than holds — holding would dribble a stale
//      half-transaction out later, interleaved with a newer one, and the
//      clapboard's CRC would then reject BOTH.

// cone_partner_present() is the TX gate clap_tx_drain consults.  Stubbed here
// (can_buses.cpp compiles in no native binary) so a test can close it mid-drain.
namespace CanBridge { static bool g_cone_partner = true; }
namespace CanBridge { bool cone_partner_present() { return g_cone_partner; } }

// Build parallel SoA arrays for a burst of `n` frames, mirroring ArgClapSend's
// layout: every frame is a CLAP_FIELD chunk except the LAST, which is the
// CLAP_COMMIT — the shape of a real slate transaction, and the reason the drain
// must be FIFO. The order witness is payload byte 0 (= the slot index), NOT the
// arbitration id: ids must never stray into 0x7EA, which is the beacon's own id
// and would make a payload frame indistinguishable from a CLAP_LINK on the wire.
static void make_burst(int n, uint32_t* ids, uint8_t* lens, uint8_t* data8) {
  for (int i = 0; i < n; ++i) {
    ids[i] = (i == n - 1) ? ClapboardCanId::COMMIT : ClapboardCanId::FIELD;
    lens[i] = 8;
    for (int b = 0; b < 8; ++b) data8[i * 8 + b] = (uint8_t)(b == 0 ? i : 0);
  }
}

static void reset_tx() {
  reset_all();
  CanBridge::g_cone_partner = true;
}

TEST_CASE("a burst drains FIFO, exactly one frame per 100 Hz tick") {
  reset_tx();
  uint32_t ids[8]; uint8_t lens[8], data8[64];
  make_burst(8, ids, lens, data8);
  REQUIRE(clap_tx_enqueue_burst(ids, lens, data8, 8));
  CHECK(clap_tx_stats().queued == 8);

  // The first step also emits the beacon, so count only the 0x7EA-excluded frames.
  int drained = 0;
  for (int tick = 0; tick < 8; ++tick) {
    const int before = g_cone_n;
    clap_link_step(JbUdp::LinkState::UP);
    fake_advance(10000u);
    for (int i = before; i < g_cone_n; ++i) {
      if (g_cone[i].id == ClapboardCanId::LINK) continue;   // the beacon
      // FIFO: the nth drained frame is the nth enqueued one. This is what
      // guarantees CLAP_COMMIT lands AFTER its chunks with no ordering logic in
      // the firmware — so the commit id must appear last and nowhere else.
      CHECK(g_cone[i].buf[0] == (uint8_t)drained);
      CHECK(g_cone[i].id == (drained == 7 ? ClapboardCanId::COMMIT
                                          : ClapboardCanId::FIELD));
      CHECK(g_cone[i].len == 8);
      ++drained;
    }
    CHECK(drained == tick + 1);            // never more than one per tick
  }
  CHECK(clap_tx_stats().sent == 8);
  CHECK(clap_tx_stats().gated == 0);
}

TEST_CASE("enqueue is all-or-nothing when the ring cannot hold the whole burst") {
  reset_tx();
  // Fill the ring to within 3 slots of full without draining.
  uint32_t ids[48]; uint8_t lens[48], data8[384];
  make_burst(48, ids, lens, data8);
  REQUIRE(clap_tx_enqueue_burst(ids, lens, data8, 48));
  REQUIRE(clap_tx_enqueue_burst(ids, lens, data8, 13));   // 61 of 64 used
  CHECK(clap_tx_stats().queued == 61);
  CHECK(clap_tx_stats().dropped == 0);

  // A 4-frame burst does not fit: NOTHING is enqueued, and the refusal is counted
  // as the whole transaction rather than as the one frame that overflowed.
  CHECK_FALSE(clap_tx_enqueue_burst(ids, lens, data8, 4));
  CHECK(clap_tx_stats().queued == 61);
  CHECK(clap_tx_stats().dropped == 4);

  // A 3-frame burst still fits exactly.
  CHECK(clap_tx_enqueue_burst(ids, lens, data8, 3));
  CHECK(clap_tx_stats().queued == 64);
  CHECK(clap_tx_stats().ring_hwm == 64);
}

TEST_CASE("enqueue refuses a malformed burst without touching the ring") {
  reset_tx();
  uint32_t ids[8]; uint8_t lens[8], data8[64];
  make_burst(8, ids, lens, data8);

  CHECK_FALSE(clap_tx_enqueue_burst(ids, lens, data8, 0));           // empty
  CHECK_FALSE(clap_tx_enqueue_burst(ids, lens, data8,
                                    (uint8_t)(JbUdp::CLAP_MAX_FRAMES + 1)));
  CHECK_FALSE(clap_tx_enqueue_burst(nullptr, lens, data8, 4));       // null array
  lens[2] = 9;                                                       // illegal DLC
  CHECK_FALSE(clap_tx_enqueue_burst(ids, lens, data8, 8));
  CHECK(clap_tx_stats().queued == 0);

  // A SHORT frame is accepted: DLC 8 is the clapboard's rule and is enforced
  // host-side where frames are built, deliberately not here — protocol knowledge
  // in the bridge would make a clapboard protocol change need a Teensy reflash.
  lens[2] = 3;
  CHECK(clap_tx_enqueue_burst(ids, lens, data8, 8));
  CHECK(clap_tx_stats().queued == 8);
}

TEST_CASE("a closed gate DISCARDS queued frames rather than holding them") {
  reset_tx();
  uint32_t ids[6]; uint8_t lens[6], data8[48];
  make_burst(6, ids, lens, data8);
  REQUIRE(clap_tx_enqueue_burst(ids, lens, data8, 6));

  // Drain two frames with the gate open...
  for (int i = 0; i < 2; ++i) { clap_link_step(JbUdp::LinkState::UP); fake_advance(10000u); }
  CHECK(clap_tx_stats().sent == 2);

  // ...then the partner goes away mid-drain.  The RPC returned long ago, so this
  // CANNOT be an RPC error; it shows up here and, on the panel, as the
  // clapboard's own CRC_MISMATCH/INCOMPLETE ack.
  CanBridge::g_cone_partner = false;
  const int wire_before = g_cone_n;
  for (int i = 0; i < 4; ++i) { clap_link_step(JbUdp::LinkState::UP); fake_advance(10000u); }
  CHECK(clap_tx_stats().sent == 2);          // no further sends
  CHECK(clap_tx_stats().gated == 4);         // the remainder discarded, not held

  // The ring is now EMPTY: a stale half-transaction must never dribble out later
  // interleaved with a newer one.
  CanBridge::g_cone_partner = true;
  for (int i = 0; i < 10; ++i) { clap_link_step(JbUdp::LinkState::UP); fake_advance(10000u); }
  for (int i = wire_before; i < g_cone_n; ++i) {
    CHECK(g_cone[i].id == ClapboardCanId::LINK);   // beacons only
  }
  CHECK(clap_tx_stats().sent == 2);
}

TEST_CASE("the census partitions every frame the host handed over") {
  reset_tx();
  uint32_t ids[10]; uint8_t lens[10], data8[80];
  make_burst(10, ids, lens, data8);
  REQUIRE(clap_tx_enqueue_burst(ids, lens, data8, 10));
  for (int i = 0; i < 4; ++i) { clap_link_step(JbUdp::LinkState::UP); fake_advance(10000u); }

  const ClapTxStats s = clap_tx_stats();
  // queued == sent + gated + still-in-ring, which is what makes CLAP_DIAG
  // readable as an account rather than as four unrelated numbers.
  CHECK(s.queued == 10);
  CHECK(s.sent == 4);
  CHECK(s.gated == 0);
  CHECK(s.dropped == 0);
  CHECK(s.ring_hwm == 10);
}

TEST_CASE("the beacon keeps its 2 Hz cadence while a burst drains") {
  // The two producers share one bus and one task; neither may starve the other.
  reset_tx();
  uint32_t ids[48]; uint8_t lens[48], data8[384];
  make_burst(48, ids, lens, data8);
  REQUIRE(clap_tx_enqueue_burst(ids, lens, data8, 48));

  run_ticks(100, JbUdp::LinkState::UP);      // 1.00 s
  int beacons = 0, payloads = 0;
  for (int i = 0; i < g_cone_n; ++i) {
    if (g_cone[i].id == ClapboardCanId::LINK) ++beacons; else ++payloads;
  }
  CHECK(beacons == 2);                       // unchanged by the downlink traffic
  CHECK(payloads == 48);                     // 48 frames in 48 ticks, one each
}
