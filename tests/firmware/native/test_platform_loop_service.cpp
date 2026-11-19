// =============================================================================
//  test_platform_loop_service.cpp — compiled test of the Platform Teensy's loop()
//  decisions (loop_service.h, FW 11, IDEA-20 P2a)
// =============================================================================
//  The tilt service against a scripted fake inclinometer: healthy request answered
//  in the same pass, the failure path's one-operation-per-pass budget and its due
//  times, the explicit invalid reply and its bytes, the probe floor and back-off,
//  no probe during a firmware update, a request arriving mid-retry, and every due
//  time across the uint32 clock wrap. Plus the CAN dispatch bound.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>

#include "../../../ros_ws/src/jugglebot/Teensy_code_platform/loop_service.h"
#include "../../../ros_ws/src/jugglebot/Teensy_code_platform/imu_service.h"  // the IMU's TX reserve
#include <functional>
#include <random>

using LoopService::reached;
using LoopService::Tilt::Reply;
using LoopService::Tilt::Service;
namespace T = LoopService::Tilt;

namespace {

struct FakeIo {
  std::deque<bool> reads;   // scripted read() results; empty = fail
  std::deque<bool> probes;  // scripted probe() results; empty = fail
  int n_read = 0, n_probe = 0;
  std::vector<Reply> replies;

  bool read() {
    ++n_read;
    if (reads.empty()) return false;
    bool r = reads.front();
    reads.pop_front();
    return r;
  }
  bool probe() {
    ++n_probe;
    if (probes.empty()) return false;
    bool r = probes.front();
    probes.pop_front();
    return r;
  }
  void reply(Reply r) { replies.push_back(r); }
  int ops() const { return n_read + n_probe; }
};

// One loop pass, asserting the per-pass budget: at most one sensor operation.
void pass(Service &s, FakeIo &io, uint32_t now, bool req = false, bool fw = false) {
  const int before = io.ops();
  s.service(now, req, fw, io);
  REQUIRE(io.ops() - before <= 1);
}

uint32_t u32_le(const uint8_t *p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}

// Start times: an ordinary one and one just before the uint32 wrap of millis().
const uint32_t STARTS[] = {1000u, 0xFFFFFFFFu - 25u};

}  // namespace

TEST_CASE("reached() and earlier() are exact across the uint32 wrap") {
  CHECK(reached(5u, 5u));
  CHECK_FALSE(reached(4u, 5u));
  CHECK(reached(3u, 0xFFFFFFF0u));          // 'now' wrapped past 'at'
  CHECK_FALSE(reached(0xFFFFFFF0u, 3u));    // 'at' is past the wrap, still ahead
  CHECK(LoopService::earlier(0xFFFFFFF0u, 3u) == 0xFFFFFFF0u);
  CHECK(LoopService::earlier(3u, 0xFFFFFFF0u) == 0xFFFFFFF0u);
  CHECK(LoopService::earlier(7u, 9u) == 7u);
}

TEST_CASE("healthy sensor: the request is answered VALID in the same pass, with one read") {
  for (uint32_t t0 : STARTS) {
    Service s; FakeIo io;
    s.boot(true, t0);
    pass(s, io, t0 + 5);                    // idle pass: nothing to do
    CHECK(io.ops() == 0);
    io.reads = {true};
    pass(s, io, t0 + 6, /*req=*/true);
    CHECK(io.n_read == 1);
    REQUIRE(io.replies.size() == 1);
    CHECK(io.replies[0] == Reply::VALID);
    CHECK_FALSE(s.request_pending());
    CHECK(io.n_probe == 0);
  }
}

TEST_CASE("failing reads: retries spread across passes by their due time, then READ_FAILED") {
  for (uint32_t t0 : STARTS) {
    Service s; FakeIo io;
    s.boot(true, t0 + 0);
    uint32_t now = t0 + 5000;               // well past the boot probe floor
    pass(s, io, now, true);                 // read 1 fails
    CHECK(io.n_read == 1);
    CHECK(io.replies.empty());
    // Not due yet: no operation, however many passes run.
    for (uint32_t dt = 1; dt < T::READ_RETRY_GAP_MS; ++dt) pass(s, io, now + dt);
    CHECK(io.n_read == 1);
    now += T::READ_RETRY_GAP_MS;
    pass(s, io, now);                       // read 2 fails
    CHECK(io.n_read == 2);
    pass(s, io, now + T::READ_RETRY_GAP_MS - 1);
    CHECK(io.n_read == 2);
    now += T::READ_RETRY_GAP_MS;
    pass(s, io, now);                       // read 3 fails -> invalid reply, sensor down
    CHECK(io.n_read == T::READ_ATTEMPTS);
    REQUIRE(io.replies.size() == 1);
    CHECK(io.replies[0] == Reply::READ_FAILED);
    CHECK_FALSE(s.sensor_ok());
    CHECK(io.n_probe == 0);                 // never in the same pass as a read
    // The re-initialisation comes on the NEXT pass (the floor is long met).
    io.probes = {true};
    pass(s, io, now + 1);
    CHECK(io.n_probe == 1);
    CHECK(s.sensor_ok());
    // ...so the host's retry is answered VALID.
    io.reads = {true};
    pass(s, io, now + 2, true);
    CHECK(io.replies.back() == Reply::VALID);
  }
}

TEST_CASE("a read that recovers mid-sequence ends it VALID") {
  Service s; FakeIo io;
  s.boot(true, 0);
  io.reads = {false, true};
  pass(s, io, 100, true);
  pass(s, io, 100 + T::READ_RETRY_GAP_MS);
  REQUIRE(io.replies.size() == 1);
  CHECK(io.replies[0] == Reply::VALID);
  CHECK(s.sensor_ok());
}

TEST_CASE("absent at boot: every request gets NOT_READY at once, without a read") {
  for (uint32_t t0 : STARTS) {
    Service s; FakeIo io;
    s.boot(false, t0);
    pass(s, io, t0 + 1, true);
    REQUIRE(io.replies.size() == 1);
    CHECK(io.replies[0] == Reply::NOT_READY);
    CHECK(io.n_read == 0);
    CHECK_FALSE(s.request_pending());
  }
}

TEST_CASE("re-probe cadence: 1 s floor after boot, doubling to the 60 s ceiling") {
  for (uint32_t t0 : STARTS) {
    Service s; FakeIo io;
    s.boot(false, t0);
    std::vector<uint32_t> probe_times;
    // Run a pass every 10 ms for 6 minutes of a sensor that never answers.
    for (uint32_t dt = 0; dt <= 360000; dt += 10) {
      const int before = io.n_probe;
      pass(s, io, t0 + dt);
      if (io.n_probe != before) probe_times.push_back(dt);
    }
    // Gaps (start to start): 1, 2, 4, 8, 16, 32, 60, 60, ... s
    REQUIRE(probe_times.size() >= 8);
    CHECK(probe_times[0] == 1000);
    std::vector<uint32_t> want = {2000, 4000, 8000, 16000, 32000, 60000, 60000};
    for (size_t i = 0; i < want.size(); ++i) CHECK(probe_times[i + 1] - probe_times[i] == want[i]);
    for (size_t i = 1; i < probe_times.size(); ++i)
      CHECK(probe_times[i] - probe_times[i - 1] >= T::PROBE_MIN_GAP_MS);
  }
}

// P2 audit B1: a request used to pull the next probe forward to the 1 s floor, so a host
// polling a dead sensor drove one 103 ms probe per second.  The schedule is now the sensor's.
std::vector<uint32_t> probe_schedule(uint32_t t0, uint32_t poll_every_ms, uint32_t minutes) {
  Service s; FakeIo io;
  s.boot(false, t0);
  std::vector<uint32_t> at;
  for (uint32_t dt = 0; dt <= minutes * 60000u; dt += 10) {
    const bool req = poll_every_ms && dt % poll_every_ms == 0;
    const int before = io.n_probe;
    pass(s, io, t0 + dt, req);
    if (io.n_probe != before) at.push_back(dt);
    if (req) CHECK(io.replies.back() == Reply::NOT_READY);
  }
  return at;
}

TEST_CASE("a dead sensor polled at 10 Hz for 10 minutes is probed exactly on the unpolled schedule") {
  for (uint32_t t0 : STARTS) {
    const auto quiet = probe_schedule(t0, 0, 10);
    const auto polled = probe_schedule(t0, 100, 10);
    CHECK(polled == quiet);
    // 1, 3, 7, 15, 31, 63, then every 60 s to 543: 14 probes in 600 s (before the fix: ~590).
    CHECK(quiet.size() == 14);
  }
}

TEST_CASE("a request never brings a probe forward, even long after the last one") {
  Service s; FakeIo io;
  s.boot(false, 0);
  for (uint32_t t = 0; t <= 200000; t += 10) pass(s, io, t);   // back-off is at the ceiling
  const int probes = io.n_probe;
  const uint32_t due = s.probe_due();
  pass(s, io, 200005, true);
  CHECK(io.replies.back() == Reply::NOT_READY);
  CHECK(io.n_probe == probes);              // answered, not probed
  CHECK(s.probe_due() == due);              // and the schedule did not move
}

TEST_CASE("no probe while a firmware update session is open") {
  Service s; FakeIo io;
  s.boot(false, 0);
  for (uint32_t t = 0; t <= 120000; t += 10) pass(s, io, t, false, /*fw=*/true);
  CHECK(io.n_probe == 0);
  pass(s, io, 120010, true, true);          // requests are still answered
  CHECK(io.replies.back() == Reply::NOT_READY);
  CHECK(io.n_probe == 0);
  pass(s, io, 120020, false, false);        // session closed: the overdue probe runs
  CHECK(io.n_probe == 1);
}

TEST_CASE("a request arriving during a failed retry sequence joins it, never restarts it") {
  Service s; FakeIo io;
  s.boot(true, 0);
  pass(s, io, 5000, true);                  // read 1 fails
  pass(s, io, 5003, true);                  // a second request mid-sequence
  CHECK(io.n_read == 1);                    // not due: no extra read
  pass(s, io, 5010, true);                  // read 2 fails; another request
  pass(s, io, 5020);                        // read 3 fails
  CHECK(io.n_read == 3);
  REQUIRE(io.replies.size() == 1);          // one reply answers all of them
  CHECK(io.replies[0] == Reply::READ_FAILED);
  // Requests every pass forever cannot postpone the reply past the 3rd read.
  Service s2; FakeIo io2;
  s2.boot(true, 0);
  for (uint32_t t = 5000; t <= 5000 + 2 * T::READ_RETRY_GAP_MS; ++t) pass(s2, io2, t, true);
  CHECK(io2.n_read == T::READ_ATTEMPTS);
  REQUIRE(io2.replies.size() >= 1);
  CHECK(io2.replies[0] == Reply::READ_FAILED);
}

TEST_CASE("worst-case reply latency of the failure path is two retry gaps") {
  for (uint32_t t0 : STARTS) {
    Service s; FakeIo io;
    s.boot(true, t0 - 100000);
    uint32_t t = t0;
    s.service(t, true, false, io);
    while (io.replies.empty()) { ++t; pass(s, io, t); }
    CHECK(t - t0 == 2 * T::READ_RETRY_GAP_MS);
  }
}

TEST_CASE("reply bytes: VALID is two f32 LE; invalid is two quiet NaNs carrying the reason") {
  uint8_t b[8];
  T::encode_valid(0.05f, -0.07f, b);
  float x, y;
  std::memcpy(&x, b, 4); std::memcpy(&y, b + 4, 4);
  CHECK(x == 0.05f); CHECK(y == -0.07f);

  T::encode_invalid(Reply::NOT_READY, b);
  CHECK(u32_le(b) == 0x7FC00000u);
  CHECK(u32_le(b + 4) == 0x7FC00001u);
  std::memcpy(&x, b, 4); std::memcpy(&y, b + 4, 4);
  CHECK(std::isnan(x)); CHECK(std::isnan(y));

  T::encode_invalid(Reply::READ_FAILED, b);
  CHECK(u32_le(b) == 0x7FC00000u);
  CHECK(u32_le(b + 4) == 0x7FC00002u);
  std::memcpy(&y, b + 4, 4);
  CHECK(std::isnan(y));
}

// ---------------------------------------------------------------------------------------------
//  CAN dispatch and the TX mailboxes (P2 audit B2, T1, T2)
// ---------------------------------------------------------------------------------------------
namespace Can = LoopService::Can;
using LoopService::CAN_FRAMES_PER_PASS;

static_assert(Can::DISPATCH_MIN_FREE_TX == 5, "2 for the dispatched frame's handler + 3 after the dispatch");
static_assert(ImuService::TX_RESERVE_MAILBOXES >= Can::WRITES_AFTER_DISPATCH_MAX + 1,
              "the IMU leaves room for the writers that run without a dispatch, plus one");

TEST_CASE("TX mailboxes: the pure formula equals FlexCAN_T4's mailboxOffset() for every MAXMB, FEN, RFFN") {
  // FlexCAN_T4.tpp mailboxOffset() (framework-arduinoteensy 1.160; pinned by test_platform_sketch_source.py).
  auto lib_offset = [](uint32_t maxmb, bool fen, uint32_t rffn) -> uint32_t {
    if (!fen) return 0;
    uint32_t remaining = maxmb - 6 - (((rffn & 0xF) + 1) * 2);
    if (maxmb < (6 + (((rffn & 0xF) + 1) * 2))) remaining = 0;
    return uint8_t(maxmb - remaining);
  };
  std::mt19937 rng(7);
  for (uint32_t maxmb = 1; maxmb <= 64; ++maxmb)
    for (int fen = 0; fen <= 1; ++fen)
      for (uint32_t rffn = 0; rffn <= 15; ++rffn) {
        const uint32_t off = lib_offset(maxmb, fen, rffn);
        CHECK(Can::mailbox_offset(maxmb, fen, rffn) == off);
        uint8_t codes[64];
        uint32_t want = 0;
        for (uint32_t i = 0; i < 64; ++i) {
          codes[i] = uint8_t(rng() % 16);
          if (i >= off && i < maxmb && codes[i] == Can::CODE_TX_INACTIVE) ++want;
        }
        CHECK(Can::free_tx_mailboxes(maxmb, fen, rffn, codes) == want);
      }
  // The sketch: setMaxMB(16), enableFIFO(), default RFFN 0 -> MB 8-15 transmit, 8 mailboxes.
  CHECK(Can::mailbox_offset(16, true, 0) == 8);
  uint8_t codes[64];
  std::memset(codes, Can::CODE_TX_INACTIVE, sizeof codes);   // FIFO area bytes look "free" too
  CHECK(Can::free_tx_mailboxes(16, true, 0, codes) == 8);    // and are never counted
}

TEST_CASE("dispatch policy: one events() per pass during an update; gated and bounded otherwise") {
  auto ret = [](uint32_t rx_left, uint32_t tx_left) { return (uint64_t(rx_left) << 12) | tx_left; };
  // Firmware update open: the first call, whatever the mailboxes and the queue say; never a second.
  CHECK(Can::may_dispatch(0, 0, true, 0, 9));
  CHECK_FALSE(Can::may_dispatch(1, ret(200, 0), true, 8, 0));
  // Otherwise: never with a frame in the software queue, never below 5 free mailboxes.
  CHECK(Can::may_dispatch(0, 0, false, Can::DISPATCH_MIN_FREE_TX, 0));
  CHECK_FALSE(Can::may_dispatch(0, 0, false, Can::DISPATCH_MIN_FREE_TX - 1, 0));
  CHECK_FALSE(Can::may_dispatch(0, 0, false, 8, 1));
  CHECK_FALSE(Can::may_dispatch(1, ret(0, 0), false, 8, 0));   // nothing left to dispatch
  CHECK_FALSE(Can::may_dispatch(1, ret(0, 5), false, 8, 0));   // a TX backlog alone does not loop
  CHECK(Can::may_dispatch(1, ret(1, 0), false, 8, 0));
  CHECK_FALSE(Can::may_dispatch(CAN_FRAMES_PER_PASS, ret(1, 0), false, 8, 0));
}

namespace {

/*  FlexCAN_T4's TX side as this sketch uses it (FlexCAN_T4.tpp write(), events(),
 *  flexcan_interrupt()): 8 TX mailboxes, write() falling back to the software queue,
 *  events()' frame.mb == -1 loop (one copy of the head frame and one pop per free mailbox)
 *  and the TX-complete ISR's one-for-one refill.  events() really returns
 *  (rx << 12) | tx (a pytest pins that line in the library source). */
struct FakeCan {
  static constexpr uint32_t MB = 8;
  uint32_t pending = 0;              // mailboxes holding a frame not yet on the wire
  std::deque<int> swq;               // the software queue
  std::deque<int> rx;                // received frames, by kind
  uint32_t entered_swq = 0, dup_copies = 0, empty_pops = 0, events_with_swq = 0;
  uint32_t free_tx() const { return MB - pending; }
  void write() {
    if (pending < MB) ++pending;
    else { swq.push_back(1); ++entered_swq; }
  }
  uint64_t events(const std::function<void(int)>& cb) {
    if (!rx.empty()) { const int k = rx.front(); rx.pop_front(); cb(k); }
    if (!swq.empty()) {
      ++events_with_swq;
      const uint32_t f = free_tx();
      for (uint32_t i = 0; i < f; ++i) {
        ++pending;
        if (i) ++dup_copies;
        if (swq.empty()) ++empty_pops; else swq.pop_front();
      }
    }
    return (uint64_t(rx.size()) << 12) | swq.size();
  }
  void complete(uint32_t n) {          // n of our frames leave the bus; the ISR refills one-for-one
    for (; n && pending; --n) {
      --pending;
      if (!swq.empty()) { swq.pop_front(); ++pending; }
    }
  }
  void complete_irq_masked(uint32_t n) { for (; n && pending; --n) --pending; }   // e.g. a flash erase
};

// Received-frame kinds and what the sketch's canSniff writes for each.
enum Kind : int { QUIET = 0, ONE_REPLY = 1, FAN_CMD = 2, TILT_REQ = 3, BEGIN = 4 };

/*  loop() as the sketch runs it: dispatch, tilt reply, traffic report (500 ms), the IMU (one
 *  frame through tx_may_place when it has one), Hand fan status (1000 ms). */
struct Sketch {
  FakeCan can;
  bool fw_open = false, old_policy = false, imu_busy = true, tilt_req = false;
  uint32_t now_ms = 0, next_report = 0, next_fan = 0, calls = 0;
  void pass() {
    auto cb = [&](int k) {
      if (k == ONE_REPLY || k == BEGIN) can.write();
      if (k == FAN_CMD) { can.write(); can.write(); }
      if (k == TILT_REQ) tilt_req = true;
      if (k == BEGIN) fw_open = true;
    };
    calls = 0;
    uint64_t ev = 0;
    if (old_policy) {                  // FW 11 before the audit: 32 frames, no TX check
      do { ev = can.events(cb); ++calls; } while (calls < CAN_FRAMES_PER_PASS && (ev >> 12));
    } else {
      while (Can::may_dispatch(calls, ev, fw_open, can.free_tx(), uint32_t(can.swq.size()))) {
        ev = can.events(cb);
        ++calls;
      }
    }
    if (tilt_req) { can.write(); tilt_req = false; }
    if (reached(now_ms, next_report)) { can.write(); next_report = now_ms + 500; }
    if (imu_busy && ImuService::tx_may_place(can.free_tx(), uint32_t(can.swq.size()))) can.write();
    if (reached(now_ms, next_fan)) { can.write(); next_fan = now_ms + 1000; }
  }
};

}  // namespace

TEST_CASE("outside an update no RX pattern puts a frame in FlexCAN's software queue while our frames move") {
  for (uint32_t seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    Sketch k;
    uint32_t stall_left = 0;
    for (uint32_t i = 0; i < 100000; ++i) {          // 10 s of 100 us passes
      k.now_ms = i / 10;
      for (uint32_t n = rng() % 9; n; --n) k.can.rx.push_back(int(rng() % 4));   // up to 8 frames per pass
      if (k.can.rx.size() > 256) k.can.rx.resize(256);                          // FlexCAN's RX queue
      if (stall_left) --stall_left;
      else if (rng() % 500 == 0) stall_left = rng() % 3000;                     // stalls up to 300 ms
      else k.can.complete(rng() % 3);
      k.pass();
    }
    CHECK(k.can.entered_swq == 0);
    CHECK(k.can.events_with_swq == 0);
  }
}

TEST_CASE("the model fails the pre-audit policy: 32 replies in one pass overflow 8 mailboxes, events() duplicates") {
  Sketch k;
  k.old_policy = true;
  k.imu_busy = false;
  for (int i = 0; i < 40; ++i) k.can.rx.push_back(ONE_REPLY);   // e.g. a BAD_SEQ/BAD_STATE run
  k.pass();
  CHECK(k.calls == CAN_FRAMES_PER_PASS);
  CHECK(k.can.entered_swq > 0);
  k.can.complete_irq_masked(3);
  k.pass();
  CHECK(k.can.dup_copies > 0);
  // The same frames under the policy: dispatch stops at 5 free, nothing queued.
  Sketch n;
  n.imu_busy = false;
  for (int i = 0; i < 40; ++i) n.can.rx.push_back(ONE_REPLY);
  n.pass();
  CHECK(n.calls == 4);                                           // 8, 7, 6, 5 free before each
  CHECK(n.can.entered_swq == 0);
}

TEST_CASE("during an update: exactly one events() per pass, as FW 10, from the BEGIN on") {
  Sketch k;
  k.imu_busy = false;
  for (int i = 0; i < 3; ++i) k.can.rx.push_back(QUIET);
  k.can.rx.push_back(BEGIN);
  for (int i = 0; i < 200; ++i) k.can.rx.push_back(ONE_REPLY);  // a BAD_SEQ run, no mailbox ever frees
  k.pass();
  CHECK(k.calls == 4);                                           // the dispatch stops at the BEGIN
  CHECK(k.fw_open);
  for (uint32_t i = 1; i < 300; ++i) {
    k.now_ms = i;
    k.pass();
    CHECK(k.calls == 1);
  }
  // FW 10's exposure, unchanged: with every mailbox full its replies reach the software queue.
  CHECK(k.can.entered_swq > 0);
}

TEST_CASE("a bus that never takes our frames: the periodic writers queue (as FW 10), events() never meets the queue") {
  for (uint32_t seed = 1; seed <= 5; ++seed) {
    std::mt19937 rng(seed);
    Sketch k;
    for (uint32_t i = 0; i < 50000; ++i) {
      k.now_ms = i / 10;
      for (uint32_t n = rng() % 4; n; --n) k.can.rx.push_back(int(rng() % 4));
      if (k.can.rx.size() > 256) k.can.rx.resize(256);
      k.pass();
    }
    CHECK(k.can.entered_swq > 0);                                // report 2 Hz + fan 1 Hz, as FW 10
    CHECK(k.can.events_with_swq == 0);                           // but no duplicate / empty-pop path
    CHECK(k.can.dup_copies == 0);
    CHECK(k.can.empty_pops == 0);
  }
}
