// =============================================================================
//  test_platform_imu_diag.cpp — the Platform IMU's DIAGNOSTIC build (-DPLATFORM_IMU_DIAG=1)
// =============================================================================
//  Compiles imu_service.h with PLATFORM_IMU_DIAG=1 and checks the repurposed IMU_STATUS bytes
//  against the decode table in Teensy_code_platform.ino ("IMU DIAGNOSTIC BUILD"): temp_c (byte 2)
//  is the page 0..7, sync_peak (byte 5) is A, fill_s (byte 6) is B. The listener decodes
//  hardware captures with this table, so a drift between it and the code fails here.
//  Logbook: 2026-11-20-platform-imu-i2c-error-bursts.

#define PLATFORM_IMU_DIAG 1
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <array>
#include <cstring>
#include <map>
#include <vector>

#include "../../../ros_ws/src/jugglebot/Teensy_code_platform/imu_service.h"

using namespace ImuService;
namespace S = PlatformImuState;

namespace {

struct FakeIo {
  uint32_t t_us = 0, stream_start_us = 0, consumed = 0;
  bool configured = false;
  int init_left = 0, pass_txns = 0, recovers = 0;
  // Scripted failures: the next N ops of that kind fail with this DiagOp.
  int fail_count = 0, fail_burst = 0, fail_init = 0;
  bool bad_header = false, force_ovf = false;
  DiagOp next, last;

  uint32_t produced() const { return configured ? (t_us - stream_start_us) / 1000 : 0; }
  void init_start() { init_left = 3; }
  int init_step(uint32_t now) {
    pass_txns++;
    last = DiagOp{};
    if (fail_init > 0) { fail_init--; last = next; return -2; }
    if (--init_left > 0) return INIT_PENDING;
    configured = true; stream_start_us = now; consumed = 0;
    return 0;
  }
  int fifo_count(bool* ovf) {
    pass_txns++;
    last = DiagOp{};
    if (fail_count > 0) { fail_count--; last = next; return -3; }
    uint32_t n = produced() - consumed;
    *ovf = force_ovf;
    if (force_ovf) { force_ovf = false; last.count = 128; last.int_status = 0x02; }
    return int(n > 120 ? 120 : n);
  }
  int read_burst(Pkt* out, int n) {
    pass_txns++;
    last = DiagOp{};
    if (fail_burst > 0) { fail_burst--; last = next; return -3; }
    if (bad_header) { bad_header = false; last = next; return ERR_BAD_PACKET; }
    for (int i = 0; i < n; i++) { for (int j = 0; j < 6; j++) out[i].s[j] = int16_t(j); out[i].temp = 10; consumed++; }
    return n;
  }
  int flush() { pass_txns++; last = DiagOp{}; consumed = produced(); return 0; }
  int check_who() { pass_txns++; last = DiagOp{}; return 0; }
  void bus_recover() { pass_txns++; recovers++; }
  void diag_op(DiagOp* d) { *d = last; }
  void stamp(Stamp* s) { s->local_us = t_us; s->wall_us = t_us; s->wall_valid = true; }
  void header_info(ImuCapture::HeaderInfo* h) { h->odr_hz = 1000; }
};

struct FakeTx {
  std::vector<std::array<uint8_t, 8>> status;
  bool place(uint32_t id, const uint8_t* d) {
    if (id == PlatformCanId::IMU_STATUS) { std::array<uint8_t, 8> a; std::memcpy(a.data(), d, 8); status.push_back(a); }
    return true;
  }
};

struct Rig {
  int16_t buf[3000 * 6];
  ImuCapture::Anchor anchors[64];
  ImuCapture::Capture cap;
  Service svc;
  FakeIo io;
  FakeTx tx;
  uint32_t t_us = 0;
  Rig() {
    cap.init(buf, 3000, anchors, 64, 7u);
    svc.init(&cap, 7u, 0);
  }
  void pass() {
    io.t_us = t_us;
    io.pass_txns = 0;
    svc.pass(t_us, t_us / 1000, false, io, tx);
    CHECK(io.pass_txns <= 1);
  }
  void run_ms(uint32_t ms) { const uint32_t end = t_us + ms * 1000; while (int32_t(t_us - end) < 0) { pass(); t_us += 500; } }
  // The latest (A, B) of every page seen in the last 8 STATUS frames.
  std::map<int, std::pair<uint8_t, uint8_t>> pages() const {
    std::map<int, std::pair<uint8_t, uint8_t>> m;
    const size_t n = tx.status.size();
    for (size_t i = n > 8 ? n - 8 : 0; i < n; i++) m[tx.status[i][2]] = {tx.status[i][5], tx.status[i][6]};
    return m;
  }
};

}  // namespace

TEST_CASE("diag pages cycle 0..7 and keep state, flags, overflows, errors and boot in place") {
  Rig r;
  r.run_ms(BOOT_SETTLE_MS + 20000);
  REQUIRE(r.svc.state() == S::RUNNING);
  REQUIRE(r.tx.status.size() >= 16);
  for (size_t i = 1; i < r.tx.status.size(); i++)
    CHECK(r.tx.status[i][2] == uint8_t((r.tx.status[i - 1][2] + 1) & 7));
  auto p = r.pages();
  CHECK(p.size() == 8);
  CHECK(p[0].first == DK_NONE);
  CHECK(r.tx.status.back()[0] == S::RUNNING);
  CHECK(r.tx.status.back()[4] == 0);       // i2c_errors
  CHECK(r.tx.status.back()[7] == 7);       // boot byte
  CHECK(p[6].second >= 1);                 // the 0.5 ms pass gap rounds up to 1 ms
  CHECK(p[6].second <= 1);
}

TEST_CASE("a FIFO_COUNT failure: kind, stage, Wire code, MSR flags, duration and the per-kind count") {
  Rig r;
  r.run_ms(BOOT_SETTLE_MS + 2000);
  REQUIRE(r.svc.state() == S::RUNNING);
  r.io.next = DiagOp{};
  r.io.next.stage = 2; r.io.next.code = 1; r.io.next.requested = 3; r.io.next.msr_flags = 0x01 | 0x20; r.io.next.dur_us = 120;
  r.io.fail_count = 1;
  r.run_ms(10);
  CHECK(r.svc.state() == S::RUNNING);    // FW 12: recovered and counted again, no BACKOFF
  r.run_ms(12000);
  CHECK(r.svc.state() == S::RUNNING);
  auto p = r.pages();
  CHECK(p[0].first == DK_COUNT);  CHECK(p[0].second == 2);
  CHECK(p[1].first == 1);         CHECK(p[1].second == 3);
  CHECK(p[2].first == 0x21);      CHECK(p[2].second == 2);   // 120 us -> 2 x 100 us, rounded up
  CHECK(p[3].first == 1);         CHECK(p[3].second == 0);
  CHECK(p[4].first == 0);         CHECK(p[4].second == 0);
  CHECK(p[5].first == 0);         CHECK(p[5].second == 1);   // one bus recovery before the retried count
  CHECK(r.io.recovers == 1);
}

TEST_CASE("a burst failure, a bad header, a probe failure, an overflow and a long pass each land on their page") {
  Rig r;
  r.run_ms(BOOT_SETTLE_MS + 2000);
  REQUIRE(r.svc.state() == S::RUNNING);
  r.io.next = DiagOp{};
  r.io.next.stage = 1; r.io.next.code = 4; r.io.next.msr_flags = 0x08; r.io.next.dur_us = 60000;
  r.io.fail_burst = 1;
  r.run_ms(20);
  CHECK(r.svc.state() == S::RUNNING);    // FW 12: recovered and flushed, no BACKOFF
  r.io.fail_count = 3;                   // three failed counts in a row do go to BACKOFF
  r.run_ms(20);
  CHECK(r.svc.state() == S::BACKOFF);
  r.io.next = DiagOp{};
  r.io.next.stage = 4; r.io.next.code = 2; r.io.next.init_failed_at = 1;   // IST_PROBE_ACK
  r.io.fail_init = 1;                    // the first re-probe fails too
  r.run_ms(15000);
  REQUIRE(r.svc.state() == S::RUNNING);
  auto p = r.pages();
  CHECK(p[0].first == DK_PROBE);  CHECK(p[0].second == 4);
  CHECK(p[3].first == 3);         CHECK(p[3].second == 1);   // three count failures, one burst failure
  CHECK(p[4].second == 1);                                   // one probe failure
  CHECK(p[5].second == 1 + 2 + 2);                           // burst retry, 2 count retries, 2 re-probes
  CHECK(p[7].first == 1);
  // A bad FIFO header: kind 6, header byte and (burst n << 4 | index), counted on page 5 A.
  r.io.next = DiagOp{};
  r.io.next.code = 0xFF; r.io.next.requested = (4 << 4) | 2;
  r.io.bad_header = true;
  r.run_ms(20);
  r.io.force_ovf = true;                 // and an overflow with INT_STATUS.FIFO_FULL
  r.run_ms(20);
  r.t_us += 37000;                       // one 37 ms pass
  r.run_ms(9000);
  p = r.pages();
  CHECK(p[0].first == DK_BAD_PACKET); CHECK(p[0].second == 0);
  CHECK(p[1].first == 0xFF);          CHECK(p[1].second == 0x42);
  CHECK(p[5].first == 1);
  CHECK(p[7].second == (0x80 | 127));
  CHECK(r.svc.counters().i2c_errors == 5);   // the bad header is not an I2C error
  CHECK(r.svc.state() == S::RUNNING);
  // Page 6 B is the longest pass gap since the previous page 6: the 37.5 ms one, then back to 1.
  bool saw = false;
  for (auto& s : r.tx.status) if (s[2] == 6 && s[6] >= 37 && s[6] <= 38) saw = true;
  CHECK(saw);
}
