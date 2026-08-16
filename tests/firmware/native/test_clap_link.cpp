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
