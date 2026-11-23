// =============================================================================
//  test_platform_sync_rx.cpp — compiled test of the Platform Teensy's 0x7DD
//  clock-sync slave (sync_rx.h, FW 15, BUG-132)
// =============================================================================
//  The envelope estimator: step on the first beacon, ~25 us/s drift both ways under
//  0-2 ms bridge TX jitter, a burst of beacons delayed 50-110 ms ignored, a real
//  40 ms bridge clock step followed (forward at once, backward within 1.0 s), a
//  silence longer than the window re-locks.  The FW 14 failure replayed: a standing
//  RX-ring delay that steps from 114 ms to 49 ms (more bus traffic) moved the
//  dispatch-stamped IIR by ~65 ms; the ISR-stamped envelope moves by under 1 ms, and
//  the dispatch-delay match reads the queue delay itself.  Plus the ISR stamp ring,
//  its reader's loss count, the 10 us delay encoding and the export bytes.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <deque>

#include "../../../ros_ws/src/jugglebot/Teensy_code_platform/sync_rx.h"
#include "../../../ros_ws/src/jugglebot/Teensy_code_platform/imu_service.h"

using SyncRx::Envelope;

namespace {

constexpr uint64_t BEACON_US = 10000;              // the bridge's 100 Hz 0x7DD
constexpr int64_t OFFSET0 = 1700000000000000ll;    // wall - local at t = 0

// Deterministic 0..n-1 (an LCG: the same sequence on every host).
struct Lcg {
  uint32_t s = 12345u;
  uint32_t next(uint32_t n) { s = s * 1664525u + 1013904223u; return (s >> 8) % n; }
};

// The bridge's TX delay for one beacon: 0x7DD loses arbitration to every other id, so it
// waits 0..2 ms behind whatever is pending; about a third go straight out.
uint32_t tx_jitter(Lcg& g) { return g.next(3) == 0 ? g.next(20) : g.next(2000); }

int64_t absll(int64_t x) { return x < 0 ? -x : x; }

}  // namespace

TEST_CASE("the first beacon steps the estimate; later ones report their innovation") {
  Envelope e;
  CHECK_FALSE(e.locked());
  Envelope::Result r = e.update(5000000, OFFSET0);
  CHECK(r.first);
  CHECK(r.stepped);
  CHECK(e.estimate() == OFFSET0);
  r = e.update(5010000, OFFSET0 - 700);       // 0.7 ms late: below the envelope, nothing moves
  CHECK_FALSE(r.first);
  CHECK_FALSE(r.stepped);
  CHECK(r.innovation_us == -700);
  CHECK(e.estimate() == OFFSET0);
  r = e.update(5020000, OFFSET0 + 30);        // less delayed than anything so far: taken at once
  CHECK(r.innovation_us == 30);
  CHECK(e.estimate() == OFFSET0 + 30);
}

TEST_CASE("tracks 25 us/s of drift both ways under 0-2 ms TX jitter, within 0.1 ms") {
  for (int sign : {+1, -1}) {
    Envelope e;
    Lcg g;
    int64_t worst = 0;
    for (uint64_t k = 0; k < 12000; ++k) {    // 120 s
      const uint64_t t = 1000000 + k * BEACON_US;
      const int64_t truth = OFFSET0 + sign * int64_t(t / 40000);   // 25 us/s
      e.update(t, truth - tx_jitter(g));
      if (k > 100) worst = std::max(worst, absll(e.estimate() - truth));
    }
    CAPTURE(sign);
    CHECK(worst < 100);   // falling: <= 25 us (one window of drift) + the window's least delay
  }
}

TEST_CASE("a burst of beacons delayed 50-110 ms is ignored") {
  Envelope e;
  Lcg g;
  uint64_t t = 1000000;
  for (int k = 0; k < 300; ++k, t += BEACON_US) e.update(t, OFFSET0 - tx_jitter(g));
  const int64_t before = e.estimate();
  // 12 beacons (120 ms, the length of a tilt re-probe stall), then a run of 80 (0.8 s), each
  // delayed 50..110 ms: neither moves the estimate, because the window still holds on-time beacons.
  for (int run : {12, 80}) {
    for (int k = 0; k < run; ++k, t += BEACON_US) {
      const Envelope::Result r = e.update(t, OFFSET0 - 50000 - int64_t(g.next(60001)));
      CHECK_FALSE(r.stepped);
    }
    CHECK(absll(e.estimate() - before) < 50);
    for (int k = 0; k < 100; ++k, t += BEACON_US) e.update(t, OFFSET0 - tx_jitter(g));
    CHECK(absll(e.estimate() - before) < 50);
  }
}

TEST_CASE("a real 40 ms bridge clock step: forward on the next beacon, backward within 1.0 s") {
  SUBCASE("forward") {
    Envelope e;
    uint64_t t = 1000000;
    for (int k = 0; k < 200; ++k, t += BEACON_US) e.update(t, OFFSET0);
    const Envelope::Result r = e.update(t, OFFSET0 + 40000);
    CHECK(r.stepped);
    CHECK(e.estimate() == OFFSET0 + 40000);
  }
  SUBCASE("backward") {
    Envelope e;
    uint64_t t = 1000000;
    for (int k = 0; k < 200; ++k, t += BEACON_US) e.update(t, OFFSET0);
    const uint64_t t_step = t;
    uint64_t t_followed = 0;
    for (int k = 0; k < 200 && !t_followed; ++k, t += BEACON_US) {
      const Envelope::Result r = e.update(t, OFFSET0 - 40000);
      if (e.estimate() == OFFSET0 - 40000) { t_followed = t; CHECK(r.stepped); }
    }
    REQUIRE(t_followed != 0);
    CHECK(t_followed - t_step >= 900000);              // a shorter delayed run is ignored (above)
    CHECK(t_followed - t_step <= 1000000 + BEACON_US);  // the stated bound
  }
}

TEST_CASE("a silence longer than the window (bridge reboot, bus down) re-locks on the next beacon") {
  Envelope e;
  e.update(1000000, OFFSET0);
  e.update(1010000, OFFSET0);
  const Envelope::Result r = e.update(1010000 + 1100000, 12345);   // a rebooted bridge, its own epoch
  CHECK(r.first);
  CHECK(e.estimate() == 12345);
}

// The FW 14 estimator, as it was (handleSyncFrame): paired with micros64() at DISPATCH, IIR diff >> 3.
struct OldIir {
  bool have = false;
  int64_t off = 0;
  void update(int64_t sample) {
    if (!have) { off = sample; have = true; return; }
    off += (sample - off) >> 3;
  }
};

TEST_CASE("replay: the RX-ring delay steps 114 ms -> 49 ms; the ISR stamp moves under 1 ms, FW 14 moved ~65 ms") {
  // Beacons at 100 Hz pass a FIFO delay line (the leaked ring) before dispatch; the delay is
  // 255 / lambda: 114 ms idle, 49 ms Armed.  The ISR stamps each one on arrival (StampRing); loop()
  // feeds the envelope from the stamps and, at dispatch, matches the stamp to measure the delay.
  Envelope env;
  OldIir old;
  SyncRx::StampRing ring;
  SyncRx::Reader rd;
  Lcg g;
  struct Pending { uint64_t wall; uint64_t dispatch_at; };
  std::deque<Pending> line;
  int64_t env_before = 0, old_before = 0;
  uint32_t delay_seen_max = 0, delay_seen_min = UINT32_MAX;
  const uint64_t t_traffic = 20000000;   // the re-arm: more traffic, a shorter delay line
  for (uint64_t t = 0; t < 40000000; t += 100) {   // 100 us loop() passes, 40 s
    if (t % BEACON_US == 0) {                       // a beacon arrives: wall = local + offset - tx jitter
      const uint64_t wall = uint64_t(OFFSET0 + int64_t(t)) - tx_jitter(g);
      ring.push(wall, t);                           // the ISR stamp
      const uint64_t d = t < t_traffic ? 114000 : 49000;
      line.push_back({wall, t + d});
    }
    SyncRx::Stamp st;                               // loop(): TimeSync::service()
    while (rd.take(ring, &st)) env.update(st.rx_local_us, int64_t(st.wall_us) - int64_t(st.rx_local_us));
    while (!line.empty() && line.front().dispatch_at <= t) {   // loop(): canSniff
      old.update(int64_t(line.front().wall) - int64_t(t));
      uint64_t rx = 0;
      if (ring.find(line.front().wall, &rx) && t > 30000000) {
        delay_seen_max = std::max(delay_seen_max, uint32_t(t - rx));
        delay_seen_min = std::min(delay_seen_min, uint32_t(t - rx));
      }
      line.pop_front();
    }
    if (t == t_traffic - 100) { env_before = env.estimate(); old_before = old.off; }
  }
  CHECK(rd.lost() == 0);
  CHECK(absll(env.estimate() - env_before) < 1000);       // the requirement
  CHECK(absll(env.estimate() - OFFSET0) < 100);           // and it is right, not just steady
  CHECK(old.off - old_before > 60000);                    // FW 14: the clock jumped ~65 ms LATER
  CHECK(old.off - old_before < 70000);
  CHECK(delay_seen_min >= 49000);                         // the dispatch match reads the queue delay
  CHECK(delay_seen_max <= 49100);
}

TEST_CASE("the stamp ring overwrites the oldest and the reader counts what it missed") {
  SyncRx::StampRing ring;
  SyncRx::Reader rd;
  SyncRx::Stamp st;
  CHECK_FALSE(rd.take(ring, &st));
  for (uint64_t i = 0; i < 20; ++i) ring.push(1000 + i, i);
  CHECK(ring.written() == 20);
  REQUIRE(rd.take(ring, &st));
  CHECK(rd.lost() == 4);                 // 20 pushed into 16 slots before the first read
  CHECK(st.wall_us == 1004);
  int n = 1;
  while (rd.take(ring, &st)) ++n;
  CHECK(n == 16);
  CHECK(st.wall_us == 1019);
  uint64_t rx = 0;
  CHECK(ring.find(1019, &rx));
  CHECK(rx == 19);
  CHECK(ring.find(1004, &rx));
  CHECK_FALSE(ring.find(1003, &rx));     // overwritten: the dispatch match reports UNMATCHED
}

TEST_CASE("payload decode and the 10 us delay encoding") {
  const uint8_t b[8] = {0x01, 0x00, 0x00, 0x00, 0x40, 0x42, 0x0F, 0x00};   // 1 s + 1000000 us
  CHECK(SyncRx::decode_wall_us(b) == 2000000ull);
  CHECK(SyncRx::delay_10us(0) == 0);
  CHECK(SyncRx::delay_10us(1) == 1);
  CHECK(SyncRx::delay_10us(10) == 1);
  CHECK(SyncRx::delay_10us(11) == 2);
  CHECK(SyncRx::delay_10us(114000) == 11400);
  CHECK(SyncRx::delay_10us(700000) == 0xFFFE);
  CHECK(SyncRx::delay_10us(SyncRx::DELAY_UNMATCHED) == 0xFFFE);
}

TEST_CASE("the export: capture-scoped receive-path health in the diag image bytes 26-29") {
  ImuService::Service svc;
  uint8_t d[ImuCapture::DIAG_BYTES];
  svc.diag_bytes(d);
  CHECK((d[26] | (d[27] << 8)) == 0xFFFF);   // no beacon dispatched yet
  CHECK(d[28] == 0);
  CHECK(d[29] == 0);
  svc.sync().note_rx_delay(35);
  svc.sync().note_rx_delay(1234);
  svc.sync().note_rx_delay(90);
  svc.sync().note_ring(3, 0);
  svc.sync().note_ring(300, 2);              // a ring-full stall: depth saturates at 255
  svc.diag_bytes(d);
  CHECK((d[26] | (d[27] << 8)) == 124);
  CHECK(d[28] == 255);
  CHECK(d[29] == 2);
  svc.sync().note_rx_delay(SyncRx::DELAY_UNMATCHED);
  svc.diag_bytes(d);
  CHECK((d[26] | (d[27] << 8)) == 0xFFFE);
  svc.sync().reset_capture();
  svc.diag_bytes(d);
  CHECK((d[26] | (d[27] << 8)) == 0xFFFF);
  CHECK(d[28] == 0);
  CHECK(d[29] == 0);
}
