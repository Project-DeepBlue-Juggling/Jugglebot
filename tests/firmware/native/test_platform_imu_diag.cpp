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
  // Known-data mode: the 32-byte register block. Byte 3 changes on every read (a volatile register).
  int regs_reads = 0, drain_ops = 0, fail_regs = 0;
  int corrupt_off = -1;                  // the next read has byte corrupt_off XORed with corrupt_xor
  uint8_t corrupt_xor = 0;
  static uint8_t ref_byte(int i) { return uint8_t(i * 37 + 5); }
  int read_regs(uint8_t reg, uint8_t* buf, uint8_t n) {
    pass_txns++;
    last = DiagOp{};
    CHECK(reg == CONST_REG);
    CHECK(n == CONST_LEN);
    if (fail_regs > 0) { fail_regs--; last = next; return -3; }
    regs_reads++;
    for (int i = 0; i < n; i++) buf[i] = ref_byte(i);
    buf[3] = uint8_t(regs_reads);
    if (corrupt_off >= 0) { buf[corrupt_off] ^= corrupt_xor; corrupt_off = -1; }
    return 0;
  }

  uint32_t produced() const { return configured ? (t_us - stream_start_us) / 1000 : 0; }
  void init_start(uint16_t) { init_left = 3; }
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
    drain_ops++;
    last = DiagOp{};
    if (fail_count > 0) { fail_count--; last = next; return -3; }
    uint32_t n = produced() - consumed;
    *ovf = force_ovf;
    if (force_ovf) { force_ovf = false; last.count = 128; last.int_status = 0x02; }
    return int(n > 120 ? 120 : n);
  }
  int read_burst(Pkt* out, int n) {
    pass_txns++;
    drain_ops++;
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
  bool fw_open = false;
  Rig() {
    cap.init(buf, 3000, anchors, 64, 7u);
    svc.init(&cap, 7u, 0);
  }
  void pass() {
    io.t_us = t_us;
    io.pass_txns = 0;
    svc.pass(t_us, t_us / 1000, fw_open, io, tx);
    CHECK(io.pass_txns <= 1);
  }
  void run_ms(uint32_t ms) { const uint32_t end = t_us + ms * 1000; while (int32_t(t_us - end) < 0) { pass(); t_us += 500; } }
  // The latest (A, B) of every page seen in the last DIAG_PAGES STATUS frames.
  std::map<int, std::pair<uint8_t, uint8_t>> pages() const {
    std::map<int, std::pair<uint8_t, uint8_t>> m;
    const size_t n = tx.status.size();
    for (size_t i = n > DIAG_PAGES ? n - DIAG_PAGES : 0; i < n; i++) m[tx.status[i][2]] = {tx.status[i][5], tx.status[i][6]};
    return m;
  }
};

}  // namespace

TEST_CASE("diag pages cycle 0..23 and keep state, flags, overflows, errors and boot in place") {
  Rig r;
  r.run_ms(BOOT_SETTLE_MS + 2 * DIAG_PAGES * 1000 + 2000);
  REQUIRE(r.svc.state() == S::RUNNING);
  REQUIRE(r.tx.status.size() >= 2 * DIAG_PAGES);
  for (size_t i = 1; i < r.tx.status.size(); i++)
    CHECK(r.tx.status[i][2] == uint8_t((r.tx.status[i - 1][2] + 1) % DIAG_PAGES));
  auto p = r.pages();
  CHECK(p.size() == DIAG_PAGES);
  CHECK(p[13].first == 0);                 // known-data mode off in the plain diag build
  CHECK(p[14].first == 0);
  CHECK(p[22].first == CONST_LEN);
  CHECK(r.io.regs_reads == 0);
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
  r.run_ms(DIAG_PAGES * 1000 + 2000);    // one whole page cycle
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
  r.run_ms(DIAG_PAGES * 1000 + 5000);
  REQUIRE(r.svc.state() == S::RUNNING);
  auto p = r.pages();
  CHECK(p[0].first == DK_PROBE);  CHECK(p[0].second == 4);
  CHECK(p[3].first == 3);         CHECK(p[3].second == 1);   // three count failures, one burst failure
  CHECK(p[4].second == 1);                                   // one probe failure
  CHECK(p[5].second == 1 + 2 + 2);                           // burst retry, 2 count retries, 2 re-probes
  CHECK(p[7].first == 1);
  // A bad FIFO header: kind 6, header byte and (burst n << 4 | index), counted on page 5 A.
  r.io.next = DiagOp{};
  r.io.next.code = 0xFF; r.io.next.requested = (4 << 4) | 2; r.io.next.good_hdr = 0x68;
  r.io.bad_header = true;
  r.run_ms(20);
  r.io.force_ovf = true;                 // and an overflow with INT_STATUS.FIFO_FULL
  r.run_ms(20);
  r.t_us += 37000;                       // one 37 ms pass
  r.run_ms(DIAG_PAGES * 1000 + 2000);
  p = r.pages();
  CHECK(p[0].first == DK_BAD_PACKET); CHECK(p[0].second == 0);
  CHECK(p[1].first == 0xFF);          CHECK(p[1].second == 0x42);
  CHECK(p[5].first == 1);
  CHECK(p[7].second == (0x80 | 127));
  CHECK(p[8].first == 0xFF);          CHECK(p[8].second == 0x42);   // the bad-header log, newest first
  CHECK(p[9].first == 0);             CHECK(p[9].second == 0);
  CHECK(p[12].first == 0x68);                                      // the last good header
  CHECK(r.svc.counters().i2c_errors == 5);   // the bad header is not an I2C error
  CHECK(r.svc.state() == S::RUNNING);
  // Page 6 B is the longest pass gap since the previous page 6: the 37.5 ms one, then back to 1.
  bool saw = false;
  for (auto& s : r.tx.status) if (s[2] == 6 && s[6] >= 37 && s[6] <= 38) saw = true;
  CHECK(saw);
}

TEST_CASE("bad-header log keeps the last four, newest first") {
  Rig r;
  r.run_ms(BOOT_SETTLE_MS + 1000);
  REQUIRE(r.svc.state() == S::RUNNING);
  for (uint8_t k = 1; k <= 5; k++) {
    r.io.next = DiagOp{};
    r.io.next.code = uint8_t(0xE0 + k); r.io.next.requested = uint8_t(0x40 | k);
    r.io.bad_header = true;
    r.run_ms(30);
  }
  r.run_ms(DIAG_PAGES * 1000 + 1000);
  auto p = r.pages();
  for (int i = 0; i < 4; i++) {
    CHECK(p[8 + i].first == uint8_t(0xE5 - i));
    CHECK(p[8 + i].second == uint8_t(0x45 - i));
  }
}

TEST_CASE("known-data compare: counts bytes, transactions and flips by direction") {
  uint8_t ref[4] = {0xF0, 0x0F, 0xAA, 0x55}, mask[4] = {1, 1, 0, 1}, got[4];
  ConstStats st;
  std::memcpy(got, ref, 4);
  const_compare(ref, mask, got, 4, &st);
  CHECK(st.txns == 1); CHECK(st.bad_txns == 0); CHECK(st.bad_bytes == 0);
  got[0] = 0x70;                           // one 1 -> 0
  got[1] = 0x1F;                           // one 0 -> 1
  got[2] = 0x00;                           // masked: ignored
  const_compare(ref, mask, got, 4, &st);
  CHECK(st.txns == 2); CHECK(st.bad_txns == 1); CHECK(st.bad_bytes == 2);
  CHECK(st.flips_10 == 1); CHECK(st.flips_01 == 1);
  CHECK(st.last_exp == 0x0F); CHECK(st.last_got == 0x1F); CHECK(st.last_off == 1);
  got[1] = 0x0F; got[3] = 0xAA;            // 0x55 -> 0xAA: four of each
  const_compare(ref, mask, got, 4, &st);
  CHECK(st.flips_10 == 1 + 4 + 1); CHECK(st.flips_01 == 1 + 4);   // got[0] still 0x70
}

TEST_CASE("known-data mode: no FIFO drain, learns the reference, counts corruption, keeps the safety rules") {
  Rig r;
  r.svc.set_const_mode(true);
  r.run_ms(BOOT_SETTLE_MS - 100);
  CHECK(r.io.regs_reads == 0);             // the boot settle: no I2C
  r.run_ms(1100);
  REQUIRE(r.svc.state() == S::RUNNING);
  const auto& st = r.svc.const_stats();
  const uint32_t t0 = st.txns;
  r.run_ms(1000);
  CHECK(r.io.drain_ops == 0);              // the FIFO is never read in this mode
  CHECK(st.txns - t0 >= 499);              // one read per 2 ms
  CHECK(st.txns - t0 <= 501);
  CHECK(st.bad_txns == 0);                 // byte 3 changes every read: masked, not counted
  r.io.corrupt_off = 5;                    // ref 0xBE: bit 7 1 -> 0, bit 0 0 -> 1
  r.io.corrupt_xor = 0x81;
  r.run_ms(10);
  CHECK(st.bad_txns == 1); CHECK(st.bad_bytes == 1);
  CHECK(st.flips_10 == 1); CHECK(st.flips_01 == 1);
  CHECK(st.last_exp == 0xBE); CHECK(st.last_got == 0x3F); CHECK(st.last_off == 5);
  r.io.corrupt_off = 3;                    // a masked byte: not counted
  r.io.corrupt_xor = 0xFF;
  r.run_ms(10);
  CHECK(st.bad_bytes == 1);
  // A failed read is counted, recovered and retried; RUNNING goes on.
  r.io.fail_regs = 1;
  r.run_ms(10);
  CHECK(r.svc.state() == S::RUNNING);
  CHECK(st.fails == 1);
  CHECK(r.io.recovers == 1);
  r.run_ms(DIAG_PAGES * 1000 + 1000);
  auto p = r.pages();
  CHECK(p[13].first == 2);  CHECK(p[13].second == 1);       // measuring, one byte masked
  CHECK(p[17].first == 1);  CHECK(p[17].second == 0);       // mismatched bytes, u16
  CHECK(p[18].first == 1);  CHECK(p[19].first == 1);
  CHECK(p[20].first == 0xBE); CHECK(p[20].second == 0x3F);
  CHECK(p[21].first == 5);  CHECK(p[21].second == CONST_REG + 5);
  CHECK(p[22].first == CONST_LEN);
  CHECK(p[23].second == CONST_PERIOD_US / 100);
  const uint32_t p14 = uint32_t(p[14].first | (p[14].second << 8));   // a snapshot from <= 24 s ago
  CHECK(p14 > 0);
  CHECK(p14 <= st.txns);
  // Three failed reads in a row: BACKOFF, re-probe, and the reference is learnt again.
  r.io.fail_regs = 3;
  r.run_ms(20);
  CHECK(r.svc.state() == S::BACKOFF);
  r.run_ms(3000);
  CHECK(r.svc.state() == S::RUNNING);
  const uint32_t before = st.txns;
  r.run_ms(100);
  CHECK(st.txns > before);
  CHECK(st.bad_bytes == 1);                // the relearnt reference matches
  // FwUpdate BEGIN: FW_HALTED, no I2C while the session is open.
  const int reads = r.io.regs_reads;
  r.fw_open = true;
  r.run_ms(2000);
  CHECK(r.svc.state() == S::FW_HALTED);
  CHECK(r.io.regs_reads == reads);
}
