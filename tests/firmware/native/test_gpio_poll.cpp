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
// gpio_poll_init() zeroes the pacing stamp, so the FIRST request is due one whole
// CHECK_INTERVAL_MS after init, not immediately.
static void reset_all() {
  fake_reset();
  version_check_init();
  gpio_poll_init();
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

  fake_advance(POLL_US - 1);
  gpio_poll_step();
  CHECK(fake_sent_count() == 1);        // one tick short of the interval: still quiet
  fake_advance(1);
  gpio_poll_step();
  CHECK(fake_sent_count() == 2);        // exactly at CHECK_INTERVAL_MS
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

// ── 12. Late reply after its timeout is dropped by the pre-send invalidate ───

TEST_CASE("a reply that lands after its timeout is dropped, never published") {
  reset_all();
  seed_matching_hand_version();

  poll_round_timeout();                 // request timed out, phase back to IDLE
  const uint64_t late_wall = 0xDEADBEEFull;
  gpio_poll_record(states_for(true), late_wall, fake_mono_us());   // hand answers late

  fake_advance(POLL_US);
  gpio_poll_step();                     // invalidate (drops the late reply), then send
  CHECK(fake_sent_count() == 2);
  gpio_poll_step();                     // AWAIT: mailbox is empty, nothing to take
  const GpioPollSnapshot s = snap();
  CHECK(s.t_bridge_us != late_wall);
  CHECK(s.t_bridge_us == 0);            // the late reply was never published
}
