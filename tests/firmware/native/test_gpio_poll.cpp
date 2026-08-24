// =============================================================================
//  test_gpio_poll.cpp — compiled test of the REAL hand ball-sensor poller
// =============================================================================
//  Drives the actual compiled gpio_poll.cpp (it #includes the .cpp, so the file-
//  statics — the phase machine, the debounce counters, the mismatch latch — are
//  reachable through the module's own API) against the recording fake HAL. Its
//  sibling `tests/firmware/test_gpio_poll_xref.py` is a SOURCE SCAN: it pins the
//  symbols the poller must reference. Nothing there executes a single line, so
//  every behaviour below — which is where the plan's normative signal semantics
//  actually live — was unverified until this driver.
//
//  version_check.cpp is #included too (not linked): the Get_Version gate reads
//  its raw cache, and version_record() is the only honest way to seed it. The two
//  TUs share no symbol names, so one TU is ODR-clean (the fault_machine +
//  leg_interp precedent in build.py).
//
//  The reply seam is gpio_poll_record() called directly. can_buses.cpp's TxSdo
//  routing (axis + endpoint match) is NOT compiled here — that layer is pinned by
//  the xref's can_buses.cpp scan.
//
//  SCOPE: decision logic, not FreeRTOS/ISR concurrency (see README.md). The
//  mailbox's PRIMASK critical sections are no-ops host-side. The snapshot's
//  time_synced field IS asserted (case 11); what remains host-untested is
//  console RENDERING only — the status line's UNKNOWN-unless-synced choice and
//  the compiled-out reply on an `enabled: false` build (the Serial stub
//  discards bytes, and ENABLED is a constexpr baked in from the YAML, so that
//  arm would need a second build of the whole harness).
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>

#include "protocol_config.h"
#include "canbridge_config.h"
#include "odrive_protocol.h"
#include "axis_state.h"
#include "fake_hal.h"

#include "version_check.cpp"   // the Get_Version cache the gate reads (seed seam)
#include "gpio_poll.cpp"       // the unit under test

using namespace CanBridge;

// ── Fixture helpers ──────────────────────────────────────────────────────────

static constexpr uint64_t POLL_US    = (uint64_t)JBBallDetect::CHECK_INTERVAL_MS * 1000ull;
static constexpr uint64_t TIMEOUT_US = (uint64_t)JBBallDetect::CHECK_TIMEOUT_MS  * 1000ull;
static constexpr uint64_t STALE_US   = 2ull * (POLL_US + TIMEOUT_US);


// A raw get_gpio_states word. ACTIVE-LOW: the seated ball shorts the pin to GND,
// so HELD ⇒ the pin's bit is CLEAR. The other bits carry a recognisable pattern
// so a test can prove the word is published verbatim.
static constexpr uint32_t OTHER_BITS = 0xA5A5A500u;
static uint32_t states_for(bool held) {
  const uint32_t bit = 1u << JBBallDetect::GPIO_PIN;
  return held ? (OTHER_BITS & ~bit) : (OTHER_BITS | bit);
}

// An ODrive Get_Version reply payload: proto, hw(product,ver,variant), fw(maj,min,rev), unreleased.
static void seed_hand_version(uint8_t maj, uint8_t min, uint8_t rev, uint8_t unreleased = 0) {
  const uint8_t v[8] = {0x00, 0x03, 0x06, 0x00, maj, min, rev, unreleased};
  version_record(HAND_AXIS, v);
}
static void seed_matching_hand_version() {
  seed_hand_version((uint8_t)JBBallDetect::EXPECTED_FW[0],
                    (uint8_t)JBBallDetect::EXPECTED_FW[1],
                    (uint8_t)JBBallDetect::EXPECTED_FW[2]);
}

// Fresh poller + fresh version cache + open bus. The clock starts at 0; note that
// gpio_poll_init() zeroes the pacing grid (s_next_due_us = 0), so the FIRST request
// is due on the first step — setup() runs long before the first task_homing tick on
// the real board, so that is not a boot burst. Every case below still advances a
// full interval first, which the absolute grid then anchors on.
static void reset_all() {
  fake_reset();
  version_check_init();
  gpio_poll_init();
  // The boot default is TEMP OFF (CAN3 isolation — see gpio_poll_init). Every
  // case here exercises a RUNNING poller, so enable through the console path,
  // keeping the suite agnostic to whichever boot default is shipping.
  REQUIRE(gpio_poll_console("gpio_poll on") == true);
  fake_set_commands_allowed(true);
}

static GpioPollSnapshot snap() {
  GpioPollSnapshot s;
  gpio_poll_snapshot(s);
  return s;
}

// One complete poll round: wait out the pacing interval, let the step send the
// RxSdo, deliver the TxSdo reply stamped NOW, let the next step consume it.
static void poll_round(bool held) {
  fake_advance(POLL_US);
  gpio_poll_step();                                   // IDLE → send → AWAIT
  gpio_poll_record(states_for(held), fake_wall_us(), fake_mono_us());
  gpio_poll_step();                                   // AWAIT → apply → IDLE
}

// One poll round whose reply never arrives: send, then sit out the reply timeout.
static void poll_round_timeout() {
  fake_advance(POLL_US);
  gpio_poll_step();              // send
  fake_advance(TIMEOUT_US);
  gpio_poll_step();              // timeout → IDLE (no publish, no counter movement)
}

// The task period gpio_poll_step() is called on, and the early-fire band the
// absolute pacing grid uses. Mirrored from gpio_poll.cpp rather than hard-coded so
// a YAML/rate change moves both together.
static constexpr uint64_t TICK_US = 1000000ull / (uint64_t)HOMING_RATE_HZ;
static constexpr uint64_t SLOP_US = TICK_US / 2ull;

// Run the step at ticks spaced TICK_US apart, with a per-tick wake JITTER, and
// report how many requests went out. This is the shape the real board runs: the
// step is a 100 Hz task callback, and `now` at each callback is the tick instant
// plus that wake's scheduling delay.
struct CadenceRun {
  size_t   sends    = 0;
  uint64_t max_gap  = 0;
  uint64_t min_gap  = 0;
  size_t   gaps     = 0;
  size_t   gaps_at_interval = 0;   // gaps exactly POLL_US wide — the healthy mode
};

// Run n_ticks of the step on a TICK_US grid with a repeating per-tick WAKE JITTER
// pattern, delivering each reply rtt_us after its request, and reduce the resulting
// send instants to a gap distribution.
//
// This is the shape the real board runs, and the jitter is the whole point: the step
// is a 100 Hz task callback and `now` at each callback is the nominal tick instant
// PLUS that wake's scheduling delay. The old pacing restarted the interval from that
// jittered `now`, so the next send needed jitter_{k+2} >= jitter_k — a coin flip that
// cost a whole task period when lost.
static CadenceRun run_ticks(size_t n_ticks, uint64_t rtt_us,
                            const uint64_t* jitter_us, size_t n_jitter) {
  CadenceRun r;
  const uint64_t t0 = fake_mono_us();
  uint64_t prev_send_at = 0;
  bool have_prev = false;
  for (size_t k = 0; k < n_ticks; ++k) {
    const uint64_t j = n_jitter ? jitter_us[k % n_jitter] : 0ull;
    const uint64_t now = t0 + (uint64_t)k * TICK_US + j;
    fake_set_clock(now, now);          // absolute: jitter is not cumulative
    const size_t before = fake_sent_count();
    gpio_poll_step();
    if (fake_sent_count() == before) continue;
    if (have_prev) {
      const uint64_t gap = now - prev_send_at;
      if (r.gaps == 0 || gap > r.max_gap) r.max_gap = gap;
      if (r.gaps == 0 || gap < r.min_gap) r.min_gap = gap;
      if (gap == POLL_US) r.gaps_at_interval++;
      r.gaps++;
    }
    prev_send_at = now; have_prev = true;
    r.sends++;
    // The hand answers rtt_us later. With rtt_us < TICK_US the reply is already in
    // the mailbox when the next tick's step runs, which is the steady state on the
    // real bus (measured RTT ~ 2 ms against a 10 ms task period).
    gpio_poll_record(states_for(true), now + rtt_us, now + rtt_us);
  }
  return r;
}

// ── 1. Version UNRECEIVED: zero TX, state UNKNOWN ────────────────────────────

TEST_CASE("no Get_Version reply yet: the poller sends nothing and stays UNKNOWN") {
  reset_all();   // version cache empty ⇒ Gate::UNRECEIVED

  for (int t = 0; t < 20; ++t) { fake_advance(POLL_US); gpio_poll_step(); }

  CHECK(fake_sent_count() == 0);        // quiet: never an RxSdo before the gate passes
  const GpioPollSnapshot s = snap();
  CHECK(s.valid == false);              // UNKNOWN
  CHECK(s.stale == true);
  CHECK(s.t_bridge_us == 0);
  CHECK(s.miss_count == 0);

  // An UNRECEIVED gate is normal for seconds after boot and permanent for an
  // unpowered hand — it must never latch. Seeding the reply starts the poll.
  seed_matching_hand_version();
  fake_advance(POLL_US);
  gpio_poll_step();
  CHECK(fake_sent_count() == 1);
}

// ── 2. MATCH: TX begins, paced at CHECK_INTERVAL_MS ──────────────────────────

TEST_CASE("version MATCH: the poller requests get_gpio_states, paced at CHECK_INTERVAL_MS") {
  reset_all();
  seed_matching_hand_version();

  fake_advance(POLL_US);
  gpio_poll_step();
  REQUIRE(fake_sent_count() == 1);

  // The frame is the RxSdo function-invoke the plan specifies: OPCODE_WRITE (not
  // READ — READ does not invoke a function endpoint), the board-qualified
  // endpoint id, zero payload, addressed to the hand axis.
  const SentFrame& f = fake_sent_at(0);
  CHECK(ODrive::cmd_of(f.id) == ODriveCmd::RxSdo);
  CHECK(ODrive::axis_of(f.id) == HAND_AXIS);
  CHECK(f.buf[0] == SDO::OPCODE_WRITE);
  uint16_t ep = 0; memcpy(&ep, &f.buf[1], 2);
  CHECK(ep == EndpointId::odrive_pro_0_6_11::get_gpio_states);
  float payload = 1.0f; memcpy(&payload, &f.buf[4], 4);
  CHECK(payload == 0.0f);

  // AWAIT holds the bus: no second frame until the round closes.
  gpio_poll_step();
  gpio_poll_step();
  CHECK(fake_sent_count() == 1);

  // Reply lands → IDLE. The next request waits out the full poll interval.
  gpio_poll_record(states_for(true), fake_wall_us(), fake_mono_us());
  gpio_poll_step();
  CHECK(fake_sent_count() == 1);

  // Pacing is now an ABSOLUTE grid with a half-task-period early-fire band (the
  // grid's due instants land exactly on nominal tick instants, so without the band
  // a microsecond of negative wake jitter would push the whole cycle a tick late —
  // that is the 30 ms mode this release removes). So the poller is quiet right up
  // to one half-tick before the due instant, and fires from there on.
  fake_advance(POLL_US - SLOP_US - 1);
  gpio_poll_step();
  CHECK(fake_sent_count() == 1);        // still outside the band: quiet
  fake_advance(1);
  gpio_poll_step();
  CHECK(fake_sent_count() == 2);        // the band opens exactly a half-tick early

  // And the GRID does not move with the early fire: the next request is due one
  // whole interval after the DUE instant, not after the (early) send instant.
  // Restarting from `now` is precisely the defect being fixed.
  gpio_poll_record(states_for(true), fake_wall_us(), fake_mono_us());
  gpio_poll_step();                     // consume; may fall through, must not send
  CHECK(fake_sent_count() == 2);
  fake_advance(POLL_US - 1);            // now = due2 - SLOP - 1 + POLL_US - 1
  gpio_poll_step();
  CHECK(fake_sent_count() == 2);        // one microsecond short of the band
  fake_advance(1);
  gpio_poll_step();
  CHECK(fake_sent_count() == 3);
}

// ── 3. MISMATCH: latched — no TX ever, valid=false ───────────────────────────

TEST_CASE("version MISMATCH latches the poller parked: no RxSdo, never valid") {
  reset_all();
  // The next patch release is the dangerous neighbour: same endpoint NAME,
  // different id, and the expected id on that build answers as something else
  // entirely — plausibly, with no timeout to diagnose it.
  const uint8_t seen_rev = (uint8_t)(JBBallDetect::EXPECTED_FW[2] + 1u);
  seed_hand_version((uint8_t)JBBallDetect::EXPECTED_FW[0],
                    (uint8_t)JBBallDetect::EXPECTED_FW[1], seen_rev, /*unreleased=*/1);

  for (int t = 0; t < 20; ++t) { fake_advance(POLL_US); gpio_poll_step(); }
  CHECK(fake_sent_count() == 0);        // the refusal happens BEFORE the frame leaves
  CHECK(bool(s_mismatch) == true);      // latched (file-static; no longer on the snapshot)
  CHECK(snap().valid == false);         // mismatch surfaces to the wire as valid=false only

  // The park stash is what task_diag prints: the triple the gate actually
  // judged, plus the fw_unreleased byte (0.6.11-1 is a real hand build).
  CHECK((unsigned)s_seen_fw[0] == (unsigned)JBBallDetect::EXPECTED_FW[0]);
  CHECK((unsigned)s_seen_fw[1] == (unsigned)JBBallDetect::EXPECTED_FW[1]);
  CHECK((unsigned)s_seen_fw[2] == (unsigned)seen_rev);
  CHECK((unsigned)s_seen_fw[3] == 1u);

  // The loud line runs on task_diag and is state-free: it neither clears the
  // latch nor puts a frame on the bus, and it repeats for as long as the park holds.
  for (int t = 0; t < 3; ++t) gpio_poll_diag_step();
  CHECK(fake_sent_count() == 0);
  CHECK(bool(s_mismatch) == true);

  // Only a re-init (a bridge reboot) clears the latch.
  gpio_poll_init();
  CHECK(bool(s_mismatch) == false);
  CHECK((unsigned)s_seen_fw[2] == 0u);
}

// ── 4. Timeout is NOT a miss ─────────────────────────────────────────────────

TEST_CASE("a reply timeout advances staleness only: verdict and miss_count untouched") {
  reset_all();
  seed_matching_hand_version();

  poll_round(true);                     // establish HELD from a good reply
  REQUIRE(snap().held == true);
  REQUIRE(snap().miss_count == 0);
  const uint64_t held_stamp = snap().t_bridge_us;

  // Lose MAX_MISSING_SAMPLES + 3 replies in a row. If timeouts counted as misses
  // this would flip a healthy seated ball to EMPTY — the failure this rule exists
  // to prevent.
  for (uint32_t i = 0; i < JBBallDetect::MAX_MISSING_SAMPLES + 3u; ++i) poll_round_timeout();

  const GpioPollSnapshot s = snap();
  CHECK(s.held == true);                // verdict frozen
  CHECK(s.miss_count == 0);             // counter frozen
  CHECK(s.t_bridge_us == held_stamp);   // nothing republished
  CHECK(s.stale == true);               // staleness is the ONLY thing that moved
  CHECK(s.valid == false);              // ⇒ UNKNOWN to a consumer
}

TEST_CASE("staleness turns over exactly at the stale window, not before") {
  reset_all();
  seed_matching_hand_version();
  poll_round(true);
  REQUIRE(snap().stale == false);
  REQUIRE(snap().valid == true);

  fake_advance(STALE_US);               // exactly at the window: still fresh (> not >=)
  CHECK(snap().stale == false);
  CHECK(snap().valid == true);
  fake_advance(1);
  CHECK(snap().stale == true);
  CHECK(snap().valid == false);
}

// ── 5. Debounce asymmetry ────────────────────────────────────────────────────

TEST_CASE("debounce is asymmetric: N-1 EMPTY replies keep HELD, the Nth flips it") {
  reset_all();
  seed_matching_hand_version();

  poll_round(true);
  REQUIRE(snap().held == true);

  // MAX_MISSING_SAMPLES - 1 consecutive EMPTY good replies: the raw bit reports
  // EMPTY every time, the debounced verdict does not budge.
  for (uint32_t i = 1; i < JBBallDetect::MAX_MISSING_SAMPLES; ++i) {
    poll_round(false);
    const GpioPollSnapshot s = snap();
    CHECK(s.raw_held == false);         // raw bit tracks every sample
    CHECK(s.held == true);              // verdict does not
    CHECK(s.miss_count == (uint8_t)i);
    CHECK(s.valid == true);
  }

  // The Nth flips it.
  poll_round(false);
  CHECK(snap().held == false);
  CHECK(snap().miss_count == (uint8_t)JBBallDetect::MAX_MISSING_SAMPLES);

  // A SINGLE HELD reply restores HELD and zeroes the counter — the sliding
  // contact's failure mode is a spurious open, never a spurious short.
  poll_round(true);
  CHECK(snap().held == true);
  CHECK(snap().raw_held == true);
  CHECK(snap().miss_count == 0);
}

// ── 6. First-ever reply EMPTY ⇒ EMPTY immediately ────────────────────────────

TEST_CASE("the first-ever reply is taken at face value: EMPTY reports EMPTY at once") {
  reset_all();
  seed_matching_hand_version();

  poll_round(false);
  const GpioPollSnapshot s = snap();
  CHECK(s.valid == true);               // UNKNOWN → EMPTY, no debounce delay
  CHECK(s.held == false);
  CHECK(s.raw_held == false);
  CHECK(s.miss_count == 1);
  CHECK(s.raw_states == states_for(false));   // the word is published verbatim
}

// ── 7. Runtime off/on: a reply latched during the off-window is never published ─

TEST_CASE("a reply latched while toggled off is dropped, not published on re-enable") {
  reset_all();
  seed_matching_hand_version();

  fake_advance(POLL_US);
  gpio_poll_step();                     // request out; phase AWAIT
  REQUIRE(fake_sent_count() == 1);
  REQUIRE(snap().t_bridge_us == 0);     // nothing published yet

  REQUIRE(gpio_poll_console("gpio_poll off") == true);
  gpio_poll_step();                     // the off-step drains the state machine

  // The hand answers anyway (the request was already on the wire), and the RX
  // decode has no idea the console just toggled off.
  const uint64_t ghost_wall = 0x1122334455667788ull;
  gpio_poll_record(states_for(true), ghost_wall, fake_mono_us());

  for (int t = 0; t < 3; ++t) { fake_advance(POLL_US); gpio_poll_step(); }
  CHECK(fake_sent_count() == 1);        // toggled off ⇒ no new requests

  REQUIRE(gpio_poll_console("gpio_poll on") == true);
  fake_advance(POLL_US);
  gpio_poll_step();                     // re-enabled: a NEW request, not the ghost
  CHECK(fake_sent_count() == 2);
  gpio_poll_step();
  CHECK(snap().t_bridge_us != ghost_wall);
  CHECK(snap().t_bridge_us == 0);       // the stale reply never reached the cache
  CHECK(snap().valid == false);

  // Control: the same delivery on a live poller DOES publish, so the assertion
  // above is about the off-window and not about a broken record path.
  gpio_poll_record(states_for(true), fake_wall_us(), fake_mono_us());
  gpio_poll_step();
  CHECK(snap().valid == true);
  CHECK(snap().held == true);
}

// ── 8. Stale-gap reseed ──────────────────────────────────────────────────────

TEST_CASE("the first reply after a stale gap reseeds the debounce from the raw bit") {
  reset_all();
  seed_matching_hand_version();

  poll_round(true);
  REQUIRE(snap().held == true);

  // A long outage: the bus goes quiet well past the stale window.
  fake_set_commands_allowed(false);
  fake_advance(20ull * STALE_US);
  gpio_poll_step();
  REQUIRE(snap().valid == false);       // UNKNOWN for the whole gap
  fake_set_commands_allowed(true);

  // One EMPTY reply after recovery. Without the reseed, the pre-outage HELD
  // would survive MAX_MISSING_SAMPLES more replies (~100 ms) of an empty hand.
  poll_round(false);
  const GpioPollSnapshot s = snap();
  CHECK(s.valid == true);
  CHECK(s.held == false);               // EMPTY immediately
  CHECK(s.miss_count == 1);

  // The reseed is scoped to the GAP: a short interruption inside the stale
  // window keeps the debounce history (contrast with the case above).
  poll_round(true);
  REQUIRE(snap().held == true);
  fake_advance(STALE_US / 2);
  poll_round(false);
  CHECK(snap().held == true);           // still debouncing, not reseeded
  CHECK(snap().miss_count == 1);
}

// ── 9. Stamps come from the reply, not from the step tick ────────────────────

TEST_CASE("published stamps are the arrival stamps handed to gpio_poll_record") {
  reset_all();
  seed_matching_hand_version();

  fake_advance(POLL_US);
  gpio_poll_step();                     // request out

  // The reply arrives NOW; the step that drains it runs a whole task tick later.
  const uint64_t arrival_wall = fake_wall_us();
  const uint64_t arrival_mono = fake_mono_us();
  gpio_poll_record(states_for(true), arrival_wall, arrival_mono);
  fake_advance(10000);                  // 10 ms of scheduling latency before the drain
  gpio_poll_step();

  CHECK(snap().t_bridge_us == arrival_wall);
  CHECK(snap().t_bridge_us != fake_wall_us());   // NOT the consumption instant

  // And the staleness clock runs from the carried MONOTONIC stamp: a reply whose
  // arrival stamp is already older than the stale window reads stale the instant
  // it is applied, even though the step just ran. (The clock is pushed well past
  // the window first so the backdated stamp cannot underflow.)
  fake_advance(10ull * STALE_US);
  gpio_poll_step();
  gpio_poll_record(states_for(true), fake_wall_us(), fake_mono_us() - (STALE_US + 1));
  gpio_poll_step();
  CHECK(snap().stale == true);
  CHECK(snap().valid == false);
}

// ── 10. miss_count saturates ─────────────────────────────────────────────────

TEST_CASE("miss_count saturates at 255 instead of wrapping") {
  reset_all();
  seed_matching_hand_version();

  // 300 consecutive EMPTY good replies, each inside the stale window so the
  // reseed never fires and the counter really does run to the top.
  for (int i = 0; i < 300; ++i) poll_round(false);

  const GpioPollSnapshot s = snap();
  CHECK(s.miss_count == 255);           // not 300 % 256 == 44
  CHECK(s.held == false);
  CHECK(s.valid == true);               // saturating does not invalidate the sample
}

// ── 11. Un-anchored bridge wall clock ────────────────────────────────────────

TEST_CASE("an un-anchored wall clock marks the sample not-time-synced") {
  reset_all();
  seed_matching_hand_version();

  fake_set_time_synced(false);
  poll_round(true);
  CHECK(snap().time_synced == false);   // captured with the reply's publication
  CHECK(snap().valid == true);          // wire flags stay ORTHOGONAL (Phase 5 ANDs them)

  fake_set_time_synced(true);           // anchor lands mid-run
  CHECK(snap().time_synced == false);   // still the anchor state of the LAST good reply
  poll_round(true);
  CHECK(snap().time_synced == true);
}

// ── 12. The pre-send invalidate drops whatever is cached at the send instant ──

TEST_CASE("a reply cached with no request outstanding is dropped, never published") {
  reset_all();
  seed_matching_hand_version();

  // WHY THIS CASE CHANGED SHAPE (2026-08-24). It used to time a request out, drop
  // a straggler into the mailbox, and rely on the NEXT TICK's send to invalidate
  // it — i.e. it tested the ~1-tick window between "timeout decided" and "next
  // request sent". Consume-and-send-in-the-same-tick collapses that window to
  // nothing: after a timeout the next request goes out immediately, so a straggler
  // arriving after it is attributed to the NEW request. That hazard is unchanged
  // in KIND (it was always true one tick later) and needs a reply-to-request
  // correlator to close, which the ODrive TxSdo reply gives us no sequence for.
  // What is still a firm contract, and is what this now asserts, is the
  // invalidate itself: NOTHING cached before a send survives that send.
  poll_round(true);                     // a completed round leaves the poller IDLE
  REQUIRE(snap().held == true);
  const uint64_t good_stamp = snap().t_bridge_us;
  const size_t sent_before = fake_sent_count();

  const uint64_t ghost_wall = 0xDEADBEEFull;
  gpio_poll_record(states_for(false), ghost_wall, fake_mono_us());   // unsolicited

  fake_advance(POLL_US);
  gpio_poll_step();                     // invalidate (drops the ghost), then send
  CHECK(fake_sent_count() == sent_before + 1);
  gpio_poll_step();                     // AWAIT: mailbox is empty, nothing to take
  const GpioPollSnapshot s = snap();
  CHECK(s.t_bridge_us != ghost_wall);
  CHECK(s.t_bridge_us == good_stamp);   // still the last GOOD reply
  CHECK(s.held == true);                // the ghost's EMPTY never touched the verdict
}

// ═══════════════════════════════════════════════════════════════════════════
//  13-17. CADENCE (2026-08-24, FW 15) — the poller reaches its configured rate
// ═══════════════════════════════════════════════════════════════════════════
//  Two independent defects held the achieved rate at ~42 Hz against a configured
//  50, and BOTH have to be fixed for the cadence to close — which is why these
//  cases assert the gap DISTRIBUTION under wake jitter rather than a single
//  round trip. A single round trip passed on the old code too.

TEST_CASE("cadence: at a small RTT the poll interval is the configured interval") {
  reset_all();
  seed_matching_hand_version();

  // RTT well inside one task period — the measured steady state on the healthy
  // plant is ~2 ms against a 10 ms task tick.
  const uint64_t rtt = TICK_US / 5ull;
  const CadenceRun r = run_ticks(200, rtt, nullptr, 0);

  REQUIRE(r.gaps > 0);
  CHECK(r.min_gap == POLL_US);
  CHECK(r.max_gap == POLL_US);          // THE headline: no cycle takes longer
  CHECK(r.gaps_at_interval == r.gaps);  // every single gap is the configured one
  // Transfer function, stated as a rate: 200 ticks at TICK_US is 200/HOMING_RATE_HZ
  // seconds, and one send per POLL_US means ticks/(POLL_US/TICK_US) sends.
  CHECK(r.sends == 200u / (size_t)(POLL_US / TICK_US));
}

TEST_CASE("cadence: wake jitter no longer promotes a cycle to the next tick") {
  reset_all();
  seed_matching_hand_version();

  // THE OLD FAILURE, REPRODUCED AS A DRIVER. The old pacing restarted the interval
  // from the JITTERED `now`, so a send needed jitter_{k+2} >= jitter_k. This
  // pattern makes that false on half the cycles: a late wake followed by an early
  // one. On the old code ~38 % of cycles took 30 ms; here every gap must be 20 ms.
  const uint64_t jitter[] = { TICK_US / 4ull, 0ull, TICK_US / 3ull, 0ull, TICK_US / 8ull };
  const CadenceRun r = run_ticks(300, TICK_US / 5ull, jitter, sizeof(jitter)/sizeof(jitter[0]));

  REQUIRE(r.gaps > 0);
  // Jitter still moves each SEND INSTANT by its own wake delay, so gaps vary by at
  // most the jitter spread — but never by a whole task period, which is what the
  // 30 ms mode was.
  CHECK(r.max_gap < POLL_US + TICK_US);
  CHECK(r.min_gap > POLL_US - TICK_US);
  // And the grid does not DRIFT: over 300 ticks the send count is the interval's,
  // not the degraded one. On the old transfer function C = 10*max(2, ceil(RTT/10)+1)
  // with cycles promoted to 30 ms, this count would come out materially lower.
  const size_t ideal = 300u / (size_t)(POLL_US / TICK_US);
  CHECK(r.sends >= ideal - 1u);
  CHECK(r.sends <= ideal + 1u);
}

TEST_CASE("cadence: a round trip is consumed and re-issued without losing a tick") {
  reset_all();
  seed_matching_hand_version();

  // The AWAIT branch used to `return` after consuming, so closing a round trip cost
  // a whole task period before the next request could even be considered. Drive the
  // pathological case directly: a reply that lands only just before the next due
  // instant. The consuming step must fall through and send in the SAME tick.
  fake_advance(POLL_US);
  gpio_poll_step();                       // request 1 out
  REQUIRE(fake_sent_count() == 1);

  fake_advance(POLL_US);                  // the reply took nearly a whole interval
  gpio_poll_record(states_for(true), fake_wall_us(), fake_mono_us());
  gpio_poll_step();                       // consume AND send, one tick
  CHECK(fake_sent_count() == 2);
  CHECK(snap().held == true);             // the reply was really applied, not skipped

  // Same for the TIMEOUT exit: it is an exit from AWAIT like any other.
  fake_advance(TIMEOUT_US);
  const size_t before = fake_sent_count();
  gpio_poll_step();                       // timeout AND send, one tick
  CHECK(fake_sent_count() == before + 1);
  CHECK(snap().held == true);             // timeout is still NOT a miss
  CHECK(snap().miss_count == 0);
}

TEST_CASE("cadence: at most ONE request per tick, even after a long stall") {
  reset_all();
  seed_matching_hand_version();

  // A long refusal window: the grid falls hours behind. The catch-up guard must
  // RE-BASE rather than replay the backlog — one request per tick until it caught
  // up would burst exactly the bus this poller is one-frame-per-tick paced to spare.
  poll_round(true);
  fake_set_commands_allowed(false);
  fake_advance(3600ull * 1000000ull);     // an hour of refusals
  gpio_poll_step();
  fake_set_commands_allowed(true);

  const size_t before = fake_sent_count();
  // Ten consecutive ticks with no advance beyond the task period.
  for (int k = 0; k < 10; ++k) { fake_advance(TICK_US); gpio_poll_step(); }
  const size_t sent = fake_sent_count() - before;
  // Ten ticks is 10/HOMING_RATE_HZ seconds; at one request per POLL_US that is at
  // most 10/(POLL_US/TICK_US) requests, and never one per tick.
  CHECK(sent <= 10u / (size_t)(POLL_US / TICK_US) + 1u);
  CHECK(sent < 10u);
}

TEST_CASE("cadence: a refusal consumes a whole interval, it does not spin") {
  reset_all();
  // Version gate UNRECEIVED — the boot state, and permanent for an unpowered hand.
  // The refusal happens AFTER the schedule advances, which is what stops the gate
  // being re-evaluated at task-tick rate for the seconds (or forever) it holds.
  uint64_t before_due_ticks = 0;
  for (int k = 0; k < 40; ++k) { fake_advance(TICK_US); gpio_poll_step(); ++before_due_ticks; }
  CHECK(fake_sent_count() == 0);          // still quiet — nothing left the Teensy

  // Now the same for a DOWN BUS, where the refusal is after the version gate.
  seed_matching_hand_version();
  fake_set_commands_allowed(false);
  for (int k = 0; k < 40; ++k) { fake_advance(TICK_US); gpio_poll_step(); }
  CHECK(fake_sent_count() == 0);

  // And when the bus comes back the poller is due within one interval, not stalled
  // behind an hour of unpaid schedule.
  fake_set_commands_allowed(true);
  size_t sent = 0;
  for (int k = 0; k < (int)(POLL_US / TICK_US) + 1; ++k) {
    fake_advance(TICK_US); gpio_poll_step();
    sent = fake_sent_count();
    if (sent) break;
  }
  CHECK(sent == 1u);
}

// ═══════════════════════════════════════════════════════════════════════════
//  18-19. TRI-STATE TX (2026-08-24) — the poller's owner-delegated ruling
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("a DEFERRED request is IN FLIGHT: AWAIT is armed and the reply lands") {
  reset_all();
  seed_matching_hand_version();

  // Every send defers: no TX mailbox is free, so the frame goes into the software
  // txBuffer and transmits in order. THE RULING: that is in-flight, not failed.
  // Under the old `write() > 0` this returned false, the poller stayed IDLE, and
  // the reply that DID come back was thrown away by the next send's invalidate —
  // a sensor that silently stopped sampling under exactly the TX pressure that
  // makes the hand busy.
  fake_set_send_defer_all(true);

  fake_advance(POLL_US);
  gpio_poll_step();
  REQUIRE(fake_sent_count() == 1);        // the deferred frame IS on the bus
  CHECK(fake_last_tx_class() == (int)TxCls::POLLER);
  CHECK(s_phase == PPhase::AWAIT);        // armed, so the reply has somewhere to land

  // The RTT instrument stays armed with it: queue latency IS round-trip latency,
  // and this instrument's job is the round-trip FLOOR. A disarm here would drop the
  // sample entirely and quietly under-report the census count.
  CHECK(bool(s_rtt_req_armed) == true);

  const uint64_t rtt = TICK_US / 5ull;
  fake_advance(rtt);
  gpio_poll_record(states_for(true), fake_wall_us(), fake_mono_us());
  gpio_poll_step();
  CHECK(snap().valid == true);
  CHECK(snap().held == true);             // the deferred round trip completed normally

  GpioPollRtt c{};
  gpio_poll_rtt_take(c);
  CHECK(c.count == 1u);                   // the deferral was MEASURED, not skipped
  CHECK(c.min_us == (uint32_t)rtt);
}

TEST_CASE("a FAILED request is not in flight: IDLE holds and the RTT disarms") {
  reset_all();
  seed_matching_hand_version();

  // TxResult::FAILED is the bus-partner presence gate refusing — the one outcome in
  // which no frame exists. Nothing to await, and nothing a later reply could
  // legitimately close.
  fake_set_send_fail_index(0);

  fake_advance(POLL_US);
  gpio_poll_step();
  CHECK(fake_sent_count() == 0);          // nothing reached the bus
  CHECK(s_phase == PPhase::IDLE);         // NOT armed
  CHECK(bool(s_rtt_req_armed) == false);  // an un-sent request must never be differenced

  // A reply arriving anyway cannot be folded into the RTT census.
  gpio_poll_record(states_for(true), fake_wall_us(), fake_mono_us());
  GpioPollRtt c{};
  gpio_poll_rtt_take(c);
  CHECK(c.count == 0u);

  // And the refusal still consumed its interval: no tick-rate retry storm.
  const size_t before = fake_sent_count();
  gpio_poll_step();
  CHECK(fake_sent_count() == before);
}
