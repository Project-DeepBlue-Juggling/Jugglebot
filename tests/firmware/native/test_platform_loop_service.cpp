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

TEST_CASE("a request pulls the next probe in, but never inside the 1 s floor") {
  for (uint32_t t0 : STARTS) {
    Service s; FakeIo io;
    s.boot(false, t0);
    pass(s, io, t0 + 1000);                 // probe 1 fails; next due at +2 s gap
    REQUIRE(io.n_probe == 1);
    pass(s, io, t0 + 1200, true);           // request 200 ms after a probe
    CHECK(io.replies.back() == Reply::NOT_READY);
    CHECK(io.n_probe == 1);                 // inside the floor: no probe
    for (uint32_t t = t0 + 1210; t != t0 + 2000; t += 10) pass(s, io, t);
    CHECK(io.n_probe == 1);
    io.probes = {true};
    pass(s, io, t0 + 2000);                 // the floor, not the 3 s back-off
    CHECK(io.n_probe == 2);
    CHECK(s.sensor_ok());
  }
}

TEST_CASE("a request long after the last probe is answered NOT_READY, then probed in the same pass") {
  Service s; FakeIo io;
  s.boot(false, 0);
  for (uint32_t t = 0; t <= 200000; t += 10) pass(s, io, t);   // back-off is at the ceiling
  const int probes = io.n_probe;
  const uint32_t now = 200005;
  // Find a time far from the next scheduled probe but past the floor.
  pass(s, io, now, true);
  CHECK(io.replies.back() == Reply::NOT_READY);
  CHECK(io.n_probe == probes + 1);          // reply first, then the (rationed) probe
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

TEST_CASE("CAN dispatch: drain until empty, never more than CAN_FRAMES_PER_PASS calls") {
  using LoopService::keep_dispatching;
  using LoopService::CAN_FRAMES_PER_PASS;
  auto ret = [](uint32_t rx_left, uint32_t tx_left) { return (uint64_t(rx_left) << 12) | tx_left; };
  CHECK_FALSE(keep_dispatching(1, ret(0, 0)));       // queue empty after the first call
  CHECK_FALSE(keep_dispatching(1, ret(0, 5)));       // TX backlog alone does not loop
  CHECK(keep_dispatching(1, ret(1, 0)));
  // A full 256-frame queue: count the calls one pass makes.
  for (uint32_t queued : {0u, 1u, 31u, 32u, 33u, 256u}) {
    uint32_t left = queued, calls = 0;
    uint64_t ev;
    do {
      if (left) --left;                        // events() pops one frame if any
      ev = ret(left, 0);
      ++calls;
    } while (keep_dispatching(calls, ev));
    const uint32_t want = queued == 0 ? 1u : (queued < CAN_FRAMES_PER_PASS ? queued : CAN_FRAMES_PER_PASS);
    CHECK(calls == want);
  }
}
