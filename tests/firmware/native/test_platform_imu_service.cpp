// =============================================================================
//  test_platform_imu_service.cpp — compiled test of the Platform IMU loop() glue (FW 11, P2b)
// =============================================================================
//  Includes the Platform Teensy's own imu_service.h (which pulls in imu_proto.h and
//  imu_capture.h) and drives it exactly as the sketch's loop() does, one pass() per simulated
//  loop pass, against a simulated ICM-42688 (FakeIo: a 1 kHz stream into a 128-packet FIFO)
//  and a CAN TX model (FakeTx: free mailboxes + the software queue). Contract:
//  ros_ws/docs/platform_imu_protocol.md. The property every case also checks: no pass ever
//  issues more than one I2C transaction.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <array>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "../../../ros_ws/src/jugglebot/Teensy_code_platform/imu_service.h"

using namespace ImuService;
namespace Op = PlatformImuOp;
namespace St = PlatformImuStatus;
namespace S = PlatformImuState;

namespace {

uint16_t g16(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }
uint32_t g32(const uint8_t* p) { return uint32_t(g16(p)) | (uint32_t(g16(p + 2)) << 16); }

struct FakeIo {
  bool present = true;
  bool wall_valid = true;
  bool force_overflow = false, bad_packet = false;
  uint32_t t_us = 0;
  bool configured = false;
  uint32_t stream_start_us = 0, consumed = 0;
  int init_left = 0;
  int pass_txns = 0, max_pass_txns = 0, total_txns = 0;
  std::vector<uint32_t> txn_times_ms;
  // A bus or controller left stuck by a failed transfer: every op fails until bus_recover().
  bool wedged = false;
  int stubborn = 0;                      // recoveries that do NOT clear a wedge (it outlasts the RUNNING retries)
  int recovers = 0;
  // Scripted transient failures: the next N ops of that kind fail once each (the bus itself stays fine).
  // A failed burst has taken some bytes from the chip FIFO, as a real short read can.
  int fail_c = 0, fail_b = 0, fail_f = 0;
  std::string log;                       // one letter per bus step: r recover, i init, c count, b burst, f flush, w who

  void txn(char k = '?') { pass_txns++; total_txns++; txn_times_ms.push_back(t_us / 1000); log += k; }
  // A bus step, but no device transaction: counted against the one-step-per-pass rule, not total_txns.
  void bus_recover() { pass_txns++; recovers++; if (stubborn > 0) stubborn--; else wedged = false; log += 'r'; }
  void begin_pass() { pass_txns = 0; }
  void end_pass() { if (pass_txns > max_pass_txns) max_pass_txns = pass_txns; }
  uint32_t produced() const { return configured ? (t_us - stream_start_us) / 1000 : 0; }
  uint32_t in_fifo(bool* ovf) {
    uint32_t p = produced() - consumed;
    if (p > 128) { *ovf = true; consumed = produced() - 128; p = 128; }
    return p;
  }
  void init_start() { init_left = 4; }
  int init_step(uint32_t now) {
    txn('i');
    if (!present || wedged) return -1;
    if (--init_left > 0) return INIT_PENDING;
    configured = true; stream_start_us = now; consumed = 0;
    return 0;
  }
  int fifo_count(bool* ovf) {
    txn('c');
    if (!present || wedged) return -3;
    if (fail_c > 0) { fail_c--; return -3; }
    *ovf = false;
    uint32_t n = in_fifo(ovf);
    if (force_overflow) { *ovf = true; force_overflow = false; }
    return int(n);
  }
  int read_burst(Pkt* out, int n) {
    txn('b');
    if (!present || wedged) return -3;
    if (fail_b > 0) { fail_b--; consumed += uint32_t(n) / 2; return -3; }
    if (bad_packet) { bad_packet = false; consumed += uint32_t(n); return ERR_BAD_PACKET; }
    for (int i = 0; i < n; i++) {
      const uint32_t k = consumed++;
      for (int j = 0; j < 6; j++) out[i].s[j] = int16_t((k * 7 + j) & 0x7FFF);
      out[i].temp = 10;
    }
    return n;
  }
  int flush() {
    txn('f');
    if (!present || wedged) return -3;
    if (fail_f > 0) { fail_f--; return -3; }
    consumed = produced();
    return 0;
  }
  int check_who() { txn('w'); return present && !wedged ? 0 : -2; }
  void stamp(Stamp* s) { s->local_us = t_us; s->wall_us = uint64_t(t_us) + 5000000000ull; s->wall_valid = wall_valid; }
  void header_info(ImuCapture::HeaderInfo* h) { h->odr_hz = 1000; h->accel_lsb_per_g = 2048; h->gyro_lsb_per_dps_x10 = 328; h->fw_version = 11; h->i2c_clock_100khz = 10; }
};

struct Sent { uint32_t id; std::array<uint8_t, 8> d; uint32_t t_us; };

struct FakeTx {
  uint32_t free_mb = 8, sw_queued = 0;
  uint32_t t_us = 0;
  std::vector<Sent> sent;
  bool place(uint32_t id, const uint8_t* d) {
    if (!tx_may_place(free_mb, sw_queued)) return false;
    Sent s{id, {}, t_us};
    std::memcpy(s.d.data(), d, 8);
    sent.push_back(s);
    return true;
  }
  std::vector<Sent> of(uint32_t id) const { std::vector<Sent> v; for (auto& s : sent) if (s.id == id) v.push_back(s); return v; }
};

constexpr uint32_t CAP = 3000, ACAP = 64;

struct Rig {
  int16_t buf[CAP * 6];
  ImuCapture::Anchor anchors[ACAP];
  ImuCapture::Capture cap;
  Service svc;
  FakeIo io;
  FakeTx tx;
  uint32_t t_us = 0;
  bool fw_open = false;
  // micros() and millis() wrap independently on the board: millis = ms_base + elapsed.
  uint32_t us_base = 0, ms_base = 0;

  explicit Rig(uint32_t t0_us = 0, uint32_t t0_ms = 0, bool do_init = true) : t_us(t0_us), us_base(t0_us), ms_base(t0_ms) {
    std::memset(buf, 0x5A, sizeof buf);          // DMAMEM is not zeroed: nothing may depend on it
    std::memset(anchors, 0xA5, sizeof anchors);
    if (do_init) init();
  }
  void init() {
    cap.init(buf, CAP, anchors, ACAP, 0x12345678u);
    svc.init(&cap, 0x12345678u, now_ms());
  }
  uint32_t now_ms() const { return ms_base + (t_us - us_base) / 1000; }
  void pass() {
    io.t_us = tx.t_us = t_us;
    io.begin_pass();
    svc.pass(t_us, now_ms(), fw_open, io, tx);
    io.end_pass();
    CHECK(io.pass_txns <= 1);
  }
  void run_ms(uint32_t ms, uint32_t step_us = 500) {
    const uint32_t end = t_us + ms * 1000;
    while (int32_t(t_us - end) < 0) { pass(); t_us += step_us; }
  }
  void cmd(std::initializer_list<uint8_t> b) {
    uint8_t d[8] = {};
    uint8_t n = 0;
    for (uint8_t x : b) d[n++] = x;
    svc.cmds().push(d, 8);
  }
  std::vector<Sent> replies() const { return tx.of(PlatformCanId::IMU_REPLY); }
  const Sent* last_reply() const { auto v = replies(); static Sent s; if (v.empty()) return nullptr; s = v.back(); return &s; }
  // Run until RUNNING with samples (the boot settle is 10 s).
  void to_running(uint32_t extra_ms = 500) {
    run_ms(BOOT_SETTLE_MS + 100);
    REQUIRE(svc.state() == S::RUNNING);
    run_ms(extra_ms);
  }
  void freeze_and_wait() {
    cmd({Op::FREEZE, 1});
    for (int i = 0; i < 400 && svc.engine().freeze_pending() == false && svc.state() != S::FROZEN; i++) pass();
    for (int i = 0; i < 400 && svc.engine().freeze_pending(); i++) { pass(); t_us += 500; }
    pass();
    REQUIRE(svc.state() == S::FROZEN);
    REQUIRE(cap.header_ready());
  }
  // Run until a drain has just finished (the last bus step was its one burst: ~4 packets per
  // 4 ms drain at 1 kHz), so the next drain op is a FIFO_COUNT.
  void to_drain_end() {
    for (int i = 0; i < 100 && (io.log.empty() || io.log.back() != 'b'); i++) { pass(); t_us += 500; }
    REQUIRE(io.log.back() == 'b');
  }
  int disc_anchors() const {
    int d = 0;
    for (auto& x : anchor_records()) if (x[16] & PlatformImuAnchorFlag::DISCONTINUITY) d++;
    return d;
  }
  // Anchor records from a frozen capture.
  std::vector<std::array<uint8_t, 18>> anchor_records() const {
    std::vector<std::array<uint8_t, 18>> v;
    for (uint32_t a = 0; a < cap.n_anchors(); a++) {
      std::array<uint8_t, 18> r{};
      for (uint32_t w = 0; w < 3; w++) cap.read_word(PlatformImu::ANCHOR_SEQ_BASE + 3 * a + w, r.data() + 6 * w);
      v.push_back(r);
    }
    return v;
  }
};

}  // namespace

TEST_CASE("pure rules: TX gate, back-off, temperature, sync quality") {
  CHECK_FALSE(tx_may_place(0, 0));
  CHECK_FALSE(tx_may_place(TX_RESERVE_MAILBOXES, 0));
  CHECK(tx_may_place(TX_RESERVE_MAILBOXES + 1, 0));
  CHECK_FALSE(tx_may_place(8, 1));  // anything in the SW queue: never add to the defect path
  CHECK(backoff_ms(1) == 1000);
  CHECK(backoff_ms(2) == 2000);
  CHECK(backoff_ms(6) == 32000);
  CHECK(backoff_ms(7) == 60000);
  CHECK(backoff_ms(100) == 60000);
  CHECK(temp_c_from_fifo(0) == 25);
  CHECK(temp_c_from_fifo(10) == 30);     // 10 / 2.07 = 4.83
  CHECK(temp_c_from_fifo(-10) == 20);
  CHECK(temp_data_from_fifo(10) == 640);
  SyncQuality q;
  uint8_t p; bool s;
  q.take_period(&p, &s);
  CHECK(p == 255); CHECK_FALSE(s);
  q.note(true, 1700000000000000ll);      // the step is not an innovation
  CHECK(q.take_step());
  q.note(false, -31);
  q.note(false, 12);
  q.take_period(&p, &s);
  CHECK(s); CHECK(p == 4);               // 31 us -> 4 x 10 us, rounded up
  q.note(false, 9000000);
  q.take_period(&p, &s);
  CHECK(p == 254);
  CHECK(q.capture_peak_us() == 0xFFFF);
}

TEST_CASE("IMU absent for ever: bounded work, back-off growth to 60 s, status every second") {
  Rig r;
  r.io.present = false;
  r.run_ms(600000, 1000);
  CHECK(r.io.max_pass_txns <= 1);
  CHECK(r.svc.state() == S::PROBING);
  // Probe starts at 10, 11, 13, 17, 25, 41, 73, 133, then every 60 s: two address probes each.
  CHECK(r.io.total_txns <= 2 * 17);
  CHECK(r.io.txn_times_ms.front() >= BOOT_SETTLE_MS);
  int late = 0;
  for (uint32_t t : r.io.txn_times_ms) if (t >= 300000) late++;
  CHECK(late <= 2 * 6);                  // <= 12 failing transactions in 5 min: at 68 ms each, 0.27 % duty
  auto st = r.tx.of(PlatformCanId::IMU_STATUS);
  CHECK(st.size() >= 589); CHECK(st.size() <= 591);   // from the end of the 10 s settle
  CHECK(st.back().d[0] == S::PROBING);
  CHECK((st.back().d[1] & PlatformImuStatusFlag::SENSOR_OK) == 0);
  CHECK(st.back().d[7] == 0x78);         // boot byte
  // The first probe of the boot starts on the bus as it is; every later probe (each follows a
  // failure) is preceded by exactly one bus recovery, in a pass of its own.
  CHECK(r.io.log.front() == 'i');
  CHECK(r.io.recovers == int(r.svc.counters().probes) - 1);
  CHECK(r.svc.counters().recoveries == uint32_t(r.io.recovers));
  for (size_t k = 1; k < r.io.log.size(); k++) if (r.io.log[k] == 'r') CHECK(r.io.log[k - 1] == 'i');
  // A FREEZE with nothing captured is refused, and the board stays reachable for commands.
  r.cmd({Op::FREEZE, 1});
  r.run_ms(5);
  REQUIRE(r.last_reply());
  CHECK(r.last_reply()->d[1] == St::ERR_EMPTY);
}

TEST_CASE("healthy capture: drains every 4 ms, anchors every 100 ms, no overflow") {
  Rig r;
  r.to_running(3000);
  CHECK(r.svc.counters().fifo_overflows == 0);
  CHECK(r.cap.n_samples() > 2900);
  CHECK(r.cap.n_anchors() >= 29);
  CHECK(r.io.max_pass_txns <= 1);
  r.io.wall_valid = false;               // no 0x7DD yet ...
  r.run_ms(250);
  r.io.wall_valid = true;
  r.svc.sync().note(true, 0);            // ... then the first one steps the offset
  r.run_ms(250);
  r.freeze_and_wait();
  auto a = r.anchor_records();
  REQUIRE(a.size() >= 2);
  int invalid = 0, step = 0;
  for (auto& x : a) {
    CHECK((x[16] & PlatformImuAnchorFlag::DISCONTINUITY) == 0);
    if (x[16] & PlatformImuAnchorFlag::WALL_INVALID) invalid++;
    if (x[16] & PlatformImuAnchorFlag::SYNC_STEP) { step++; CHECK((x[16] & PlatformImuAnchorFlag::WALL_INVALID) == 0); }
  }
  CHECK(invalid >= 2);
  CHECK(step == 1);                      // exactly the first anchor after the step
  // Every anchor's index is a sample count at a drain, and its local_us is that drain's stamp.
  for (size_t i = 1; i < a.size(); i++) CHECK(g32(a[i].data()) > g32(a[i - 1].data()));
}

TEST_CASE("a slow pass (inclinometer re-probe, 103 ms) is caught up without a discontinuity") {
  Rig r;
  r.to_running(500);
  r.t_us += 103000;                      // one pass held by the SCL3300 begin()
  r.run_ms(500);
  CHECK(r.svc.counters().fifo_overflows == 0);
  r.t_us += 140000;                      // longer than the 128 ms FIFO: an overflow, marked
  r.run_ms(500);
  CHECK(r.svc.counters().fifo_overflows == 1);
  r.freeze_and_wait();
  auto a = r.anchor_records();
  int disc = 0;
  for (size_t i = 0; i < a.size(); i++) {
    if (a[i][16] & PlatformImuAnchorFlag::DISCONTINUITY) {
      disc++;
      REQUIRE(i > 0);
      CHECK((a[i - 1][16] & PlatformImuAnchorFlag::DISCONTINUITY) == 0);  // the closing anchor
    }
  }
  CHECK(disc == 1);
  CHECK(((g16(r.cap.header() + 18) & PlatformImuCaptureFlag::DISCONTINUITY) != 0));
  CHECK(g16(r.cap.header() + 46) == 1);
}

TEST_CASE("unplug mid-run: RUNNING -> BACKOFF -> PROBING -> RUNNING behind a DISCONTINUITY anchor") {
  Rig r;
  r.to_running(1000);
  const uint32_t before = r.cap.appended();
  r.io.present = false;
  r.run_ms(10);
  CHECK(r.svc.state() == S::BACKOFF);
  CHECK(r.svc.fail_streak() == 1);
  r.run_ms(4000);                        // probes fail at 1 s and 2 s more
  CHECK(r.svc.state() == S::PROBING);
  CHECK(r.svc.fail_streak() >= 2);
  r.io.present = true;
  for (int i = 0; i < 20000 && r.svc.state() != S::RUNNING; i++) { r.pass(); r.t_us += 500; }  // next probe <= 8 s away
  CHECK(r.svc.state() == S::RUNNING);
  r.run_ms(500);                         // short of the 3000-sample ring, so the gap is still retained
  CHECK(r.cap.appended() > before + 100);
  CHECK(r.io.max_pass_txns <= 1);
  r.freeze_and_wait();
  auto a = r.anchor_records();
  int disc = 0;
  for (size_t i = 0; i < a.size(); i++)
    if (a[i][16] & PlatformImuAnchorFlag::DISCONTINUITY) {
      disc++;
      CHECK(g32(a[i - 1].data()) == before);   // the closing anchor ends exactly at the gap
    }
  CHECK(disc == 1);
  r.run_ms(1100);                        // FROZEN: STATUS goes on and still reports the capture's discontinuity
  auto st = r.tx.of(PlatformCanId::IMU_STATUS);
  CHECK(st.back().d[0] == S::FROZEN);
  CHECK(((st.back().d[1] & PlatformImuStatusFlag::DISCONTINUITY) != 0));
  CHECK(st.back().d[4] >= 1);            // i2c_errors
}

TEST_CASE("a bad FIFO packet flushes and marks a discontinuity") {
  Rig r;
  r.to_running(500);
  r.io.bad_packet = true;
  r.run_ms(500);
  CHECK(r.svc.counters().bad_packets == 1);
  CHECK(r.svc.state() == S::RUNNING);
  CHECK(r.svc.counters().i2c_errors == 0);   // the transfer completed: not an I2C error, no recovery, no retry
  CHECK(r.svc.counters().recoveries == 0);
  CHECK(r.svc.counters().retries == 0);
  CHECK(r.svc.counters().fifo_overflows == 0);
  r.freeze_and_wait();
  int disc = 0;
  for (auto& x : r.anchor_records()) if (x[16] & PlatformImuAnchorFlag::DISCONTINUITY) disc++;
  CHECK(disc == 1);
}

TEST_CASE("FwUpdate BEGIN in every engine state after the settle: FW_HALTED, no I2C until the session ends") {
  struct Case { const char* name; uint8_t want_before; };
  for (int which = 0; which < 7; which++) {
    CAPTURE(which);
    Rig r;
    switch (which) {
      case 0: r.run_ms(BOOT_SETTLE_MS + 5); r.cmd({Op::DISABLE}); r.run_ms(5); break;     // DISABLED
      case 1: r.io.present = false; r.run_ms(BOOT_SETTLE_MS + 500); break;               // PROBING (between probes)
      case 2: r.to_running(300); break;                                                  // RUNNING
      case 3: r.to_running(300); r.cmd({Op::FREEZE, 1}); r.pass(); break;                // FROZEN, CRC in progress
      case 4: r.to_running(300); r.freeze_and_wait(); break;                             // FROZEN
      case 5: r.to_running(300); r.freeze_and_wait();
              r.cmd({Op::READ, uint8_t(r.cap.capture_id()), uint8_t(r.cap.capture_id() >> 8), 0, 0, 0, 1, 100});
              r.pass(); break;                                                            // UPLOADING
      case 6: r.to_running(300); r.io.present = false; r.run_ms(10); break;              // BACKOFF
    }
    const uint8_t s0 = r.svc.state();
    const uint8_t want[7] = {S::DISABLED, S::PROBING, S::RUNNING, S::FROZEN, S::FROZEN, S::UPLOADING, S::BACKOFF};
    CHECK(s0 == want[which]);
    r.io.present = true;
    r.fw_open = true;
    const int txns0 = r.io.total_txns;
    const size_t data0 = r.tx.of(PlatformCanId::IMU_DATA).size();
    r.run_ms(30000);
    CHECK(r.svc.state() == S::FW_HALTED);
    CHECK(r.io.total_txns == txns0);                             // not one I2C transaction
    CHECK(r.tx.of(PlatformCanId::IMU_DATA).size() == data0);     // the block is cut
    CHECK(r.cap.n_samples() == 0);                               // the capture is discarded
    r.cmd({Op::ENABLE});
    r.run_ms(5);
    CHECK(r.last_reply()->d[1] == St::ERR_FW_UPDATE);
    CHECK(r.tx.of(PlatformCanId::IMU_STATUS).back().d[0] == S::FW_HALTED);  // STATUS goes on, 1 Hz
    // The session is abandoned (idle timeout, failed staging): back to PROBING, then capturing.
    r.fw_open = false;
    r.run_ms(10);
    CHECK(r.svc.state() == S::PROBING);
    r.run_ms(1500);
    CHECK(r.svc.state() == S::RUNNING);
  }
}

TEST_CASE("full cycle through the glue: FREEZE -> header -> anchors -> samples -> CRCs -> RESUME") {
  Rig r;
  r.to_running(4000);                        // the 3000-sample ring wraps
  r.cmd({Op::FREEZE, 1});
  for (int i = 0; i < 200; i++) { r.pass(); r.t_us += 500; }
  auto fr = r.replies();
  REQUIRE(fr.size() == 1);
  CHECK(fr[0].d[0] == Op::FREEZE);
  CHECK(fr[0].d[1] == St::OK);
  const uint16_t id = g16(fr[0].d.data() + 2);
  CHECK(g32(fr[0].d.data() + 4) == CAP);
  std::map<uint32_t, std::array<uint8_t, 6>> got;
  auto read = [&](uint32_t first, uint32_t count) {
    r.cmd({Op::READ, uint8_t(id), uint8_t(id >> 8), uint8_t(first), uint8_t(first >> 8), uint8_t(count), uint8_t(count >> 8), 80});
    for (int i = 0; i < 4000 && (r.svc.state() != S::FROZEN || i < 2); i++) { r.pass(); r.t_us += 100; }
  };
  read(PlatformImu::HEADER_SEQ_BASE, 11);
  const uint32_t na = r.cap.n_anchors();
  for (uint32_t s = 0; s < 3 * na; s += 256) read(PlatformImu::ANCHOR_SEQ_BASE + s, std::min<uint32_t>(256, 3 * na - s));
  for (uint32_t s = 0; s < 2 * CAP; s += 256) read(s, std::min<uint32_t>(256, 2 * CAP - s));
  uint32_t last_t = 0; bool first = true;
  for (auto& f : r.tx.of(PlatformCanId::IMU_DATA)) {
    std::array<uint8_t, 6> w; std::memcpy(w.data(), f.d.data() + 2, 6);
    got[g16(f.d.data())] = w;
    if (!first) CHECK(f.t_us - last_t >= 800);   // the block's gap, 80 x 10 us
    first = false; last_t = f.t_us;
  }
  CHECK(got.size() == 11 + 3 * na + 2 * CAP);
  uint8_t hdr[66];
  for (uint32_t i = 0; i < 11; i++) std::memcpy(hdr + 6 * i, got[PlatformImu::HEADER_SEQ_BASE + i].data(), 6);
  CHECK(g32(hdr + 62) == ImuCapture::crc32(hdr, 62));
  CHECK(g16(hdr + 2) == id);
  CHECK(((g16(hdr + 18) & PlatformImuCaptureFlag::WRAPPED) != 0));
  CHECK((g16(hdr + 18) & PlatformImuCaptureFlag::NEVER_SYNCED) == 0);
  CHECK(int16_t(g16(hdr + 42)) == 640);  // temp at capture start, TEMP_DATA scale
  std::vector<uint8_t> payload;
  for (uint32_t s = 0; s < 3 * na; s++) payload.insert(payload.end(), got[PlatformImu::ANCHOR_SEQ_BASE + s].begin(), got[PlatformImu::ANCHOR_SEQ_BASE + s].end());
  for (uint32_t s = 0; s < 2 * CAP; s++) payload.insert(payload.end(), got[s].begin(), got[s].end());
  CHECK(g32(hdr + 54) == ImuCapture::crc32(payload.data(), payload.size()));
  // Retained samples are consecutive chip samples (no loss in a healthy run).
  const uint32_t k0 = uint16_t(g16(got[0].data()));
  const uint32_t k1 = uint16_t(g16(got[2].data()));
  CHECK(k1 == ((k0 + 7) & 0x7FFF));
  // RESUME: a new capture, the FIFO flushed and WHO_AM_I re-checked before any sample.
  const int t0 = r.io.total_txns;
  r.cmd({Op::RESUME, uint8_t(id), uint8_t(id >> 8)});
  r.pass();
  CHECK(r.svc.state() == S::RUNNING);
  CHECK(r.cap.n_samples() == 0);
  r.run_ms(100);
  CHECK(r.io.total_txns > t0 + 2);
  CHECK(r.cap.n_samples() > 80);
  r.cmd({Op::READ, uint8_t(id), uint8_t(id >> 8), 0, 0, 1, 0, 80});
  r.run_ms(5);
  CHECK(r.last_reply()->d[1] == St::ERR_NOT_FROZEN);
  CHECK(r.io.max_pass_txns <= 1);
}

TEST_CASE("READ pacing: zero free mailboxes queues nothing; then one DATA frame per pass at the gap") {
  Rig r;
  r.to_running(300);
  r.freeze_and_wait();
  const uint16_t id = r.cap.capture_id();
  const size_t n0 = r.tx.sent.size();
  r.tx.free_mb = 0;
  r.cmd({Op::READ, uint8_t(id), uint8_t(id >> 8), 0, 0, 20, 0, 10});  // gap 10 -> raised to the 400 us floor
  for (int i = 0; i < 2000; i++) { r.pass(); r.t_us += 100; }
  CHECK(r.tx.sent.size() == n0);         // not the reply, not a status, not a data frame
  CHECK(r.svc.state() == S::UPLOADING);
  r.tx.free_mb = TX_RESERVE_MAILBOXES;   // still the reserve: nothing
  for (int i = 0; i < 100; i++) { r.pass(); r.t_us += 100; }
  CHECK(r.tx.sent.size() == n0);
  r.tx.free_mb = TX_RESERVE_MAILBOXES + 1;
  r.tx.sw_queued = 1;                    // something in the SW queue: nothing
  for (int i = 0; i < 100; i++) { r.pass(); r.t_us += 100; }
  CHECK(r.tx.sent.size() == n0);
  r.tx.sw_queued = 0;
  r.pass();
  auto after = std::vector<Sent>(r.tx.sent.begin() + long(n0), r.tx.sent.end());
  REQUIRE(after.size() >= 1);
  CHECK(after[0].id == PlatformCanId::IMU_REPLY);   // the reply goes first
  for (int i = 0; i < 300; i++) { r.pass(); r.t_us += 100; }
  auto d = r.tx.of(PlatformCanId::IMU_DATA);
  CHECK(d.size() == 20);
  for (size_t i = 1; i < d.size(); i++) CHECK(d[i].t_us - d[i - 1].t_us >= 400);
  CHECK(r.svc.state() == S::FROZEN);
}

TEST_CASE("STATUS cadence: one per second in RUNNING, with sync figures") {
  Rig r;
  r.to_running(0);
  const size_t n0 = r.tx.of(PlatformCanId::IMU_STATUS).size();
  for (int s = 0; s < 10; s++) {
    for (int i = 0; i < 100; i++) r.svc.sync().note(false, (i % 2) ? 25 : -40);
    r.run_ms(1000);
  }
  auto st = r.tx.of(PlatformCanId::IMU_STATUS);
  CHECK(st.size() - n0 == 10);
  CHECK(st.back().d[0] == S::RUNNING);
  CHECK(((st.back().d[1] & PlatformImuStatusFlag::SENSOR_OK) != 0));
  CHECK(((st.back().d[1] & PlatformImuStatusFlag::TIME_SYNCED) != 0));
  CHECK(st.back().d[5] == 4);            // 40 us peak
  CHECK(int8_t(st.back().d[2]) == 30);
  CHECK(st.back().d[6] == CAP / 1000);   // fill_s: the (test-sized) ring is full
}

TEST_CASE("command slots: full ring drops the newest; DISABLE stops all I2C; ENABLE restarts") {
  CmdSlots c;
  uint8_t d[8] = {Op::ENABLE};
  for (int i = 0; i < CMD_SLOTS - 1; i++) CHECK(c.push(d, 1));
  CHECK_FALSE(c.push(d, 1));
  CHECK(c.dropped() == 1);
  Rig r;
  r.to_running(300);
  r.cmd({Op::DISABLE});
  r.pass();
  const int t0 = r.io.total_txns;
  r.run_ms(5000);
  CHECK(r.svc.state() == S::DISABLED);
  CHECK(r.io.total_txns == t0);
  r.cmd({Op::ENABLE});
  r.run_ms(200);
  CHECK(r.svc.state() == S::RUNNING);
}

// ---------------------------------------------------------------------------------------------
//  P2 audit: the 10 s boot settle (B3), the BEGIN latch, the pre-init ISR window, clock wraps,
//  READ while FW_HALTED (T3)
// ---------------------------------------------------------------------------------------------
TEST_CASE("boot settle: for 10 s no I2C, no frame, commands dropped; then the first probe") {
  Rig r;
  r.cmd({Op::DISABLE});
  r.run_ms(3000);
  r.cmd({Op::ENABLE});                               // ENABLE cannot start a probe early either
  r.cmd({Op::FREEZE, 1});
  r.run_ms(BOOT_SETTLE_MS - 3000 - 1);
  CHECK(r.io.total_txns == 0);
  CHECK(r.tx.sent.empty());                          // no STATUS, no reply
  CHECK_FALSE(r.svc.settled());
  CHECK(r.svc.counters().cmds_in_settle == 3);
  CHECK(r.svc.state() == S::PROBING);
  r.run_ms(200);
  CHECK(r.svc.settled());
  CHECK(r.io.total_txns >= 1);
  CHECK(r.io.txn_times_ms.front() >= BOOT_SETTLE_MS);
  CHECK(r.tx.of(PlatformCanId::IMU_STATUS).size() == 1);
  CHECK(r.replies().empty());                        // the dropped commands are never answered
}

TEST_CASE("a BEGIN seen inside the settle keeps the IMU off for the rest of that boot, even if abandoned") {
  Rig r;
  r.run_ms(4000);
  r.fw_open = true;                                  // BEGIN at 4 s
  r.run_ms(1000);
  r.fw_open = false;                                 // the session ends (abandoned or failed) at 5 s
  r.run_ms(120000, 1000);
  CHECK(r.svc.halted_for_boot());
  CHECK(r.svc.state() == S::FW_HALTED);
  CHECK(r.io.total_txns == 0);
  CHECK(r.tx.of(PlatformCanId::IMU_STATUS).back().d[0] == S::FW_HALTED);
  r.cmd({Op::ENABLE});
  r.run_ms(5);
  REQUIRE(r.last_reply());
  CHECK(r.last_reply()->d[1] == St::ERR_FW_UPDATE);
  CHECK(r.io.total_txns == 0);
  // The same BEGIN after the settle: an abandoned update returns to PROBING (spec section 4).
  Rig q;
  q.run_ms(BOOT_SETTLE_MS + 1000);
  q.fw_open = true;  q.run_ms(1000);
  q.fw_open = false; q.run_ms(3000);
  CHECK_FALSE(q.svc.halted_for_boot());
  CHECK(q.svc.state() == S::RUNNING);
}

TEST_CASE("pushes before init() (canSniff in FlexCAN's ISR during setup) survive init and are dropped by the settle") {
  Rig r(0, 0, /*do_init=*/false);
  r.cmd({Op::FREEZE, 1});                            // statically constructed: usable before init()
  r.svc.sync().note(true, 1700000000000000ll);
  r.svc.sync().note(false, 40);
  r.init();
  r.run_ms(100);
  CHECK(r.svc.counters().cmds_in_settle == 1);
  r.run_ms(BOOT_SETTLE_MS + 500);
  CHECK(r.svc.state() == S::RUNNING);
  CHECK(r.replies().empty());
  CHECK(r.svc.counters().i2c_errors == 0);
}

TEST_CASE("micros() wraps mid-capture and mid-READ: drains, anchors and frame gaps carry on") {
  Rig r(0xFFFFFFFFu - 13000000u, 777);
  r.to_running(2000);                                // ~1 s before the micros() wrap
  r.run_ms(2000);                                    // across it
  CHECK(r.t_us < 2000000u);
  CHECK(r.svc.counters().fifo_overflows == 0);
  CHECK(r.svc.counters().i2c_errors == 0);
  CHECK((r.tx.of(PlatformCanId::IMU_STATUS).back().d[1] & PlatformImuStatusFlag::DISCONTINUITY) == 0);
  CHECK(r.io.consumed >= 3400);                      // 1 kHz for ~3.6 s read out, nothing lost at the wrap
  CHECK(r.io.consumed + 8 >= r.io.produced());       // and the drain kept up across it
  Rig q(0xFFFFFFFFu - 13000000u, 777);
  q.to_running(300);
  q.freeze_and_wait();
  REQUIRE(q.t_us >= 0x80000000u);                    // still before the wrap
  while (q.t_us < 0xFFFFFFFFu - 5000u) { q.pass(); q.t_us += 1000; }   // FROZEN until 5 ms before it
  const uint32_t id = q.cap.capture_id();
  const size_t before = q.tx.of(PlatformCanId::IMU_DATA).size();
  q.cmd({Op::READ, uint8_t(id), uint8_t(id >> 8), 0, 0, 30, 0, 10});
  for (int i = 0; i < 400; i++) { q.pass(); q.t_us += 100; }
  auto d = q.tx.of(PlatformCanId::IMU_DATA);
  REQUIRE(d.size() == before + 30);
  CHECK(d.back().t_us < d[before].t_us);             // the block spans the wrap
  for (size_t i = before + 1; i < d.size(); i++) CHECK(uint32_t(d[i].t_us - d[i - 1].t_us) >= 400);
}

TEST_CASE("millis() wraps through the settle and the back-off: same probe schedule as from zero") {
  Rig a(0, 0), b(0, 0xFFFFFFFFu - 30000u);
  a.io.present = b.io.present = false;
  a.run_ms(300000, 1000);
  b.run_ms(300000, 1000);
  CHECK(b.io.total_txns == a.io.total_txns);
  CHECK(b.io.txn_times_ms == a.io.txn_times_ms);     // FakeIo logs micros-based times: identical
  CHECK(b.tx.of(PlatformCanId::IMU_STATUS).size() == a.tx.of(PlatformCanId::IMU_STATUS).size());
}

TEST_CASE("READ while FW_HALTED is refused ERR_FW_UPDATE and sends no DATA") {
  Rig r;
  r.to_running(300);
  r.freeze_and_wait();
  const uint32_t id = r.cap.capture_id();
  r.fw_open = true;
  r.pass(); r.t_us += 500;
  REQUIRE(r.svc.state() == S::FW_HALTED);
  const size_t data_before = r.tx.of(PlatformCanId::IMU_DATA).size();
  r.cmd({Op::READ, uint8_t(id), uint8_t(id >> 8), 0, 0, 5, 0, 80});
  r.run_ms(50);
  REQUIRE(r.last_reply());
  CHECK(r.last_reply()->d[1] == St::ERR_FW_UPDATE);
  CHECK(r.tx.of(PlatformCanId::IMU_DATA).size() == data_before);
}

TEST_CASE("a bus or controller wedged by a failed transfer heals without a reboot (unconditional recovery)") {
  // FW 11 on the robot, 2026-11-19 overnight: after repeated transient errors the engine sat in
  // PROBING with SENSOR_OK clear until the Teensy was rebooted, while the chip itself was fine.
  // Model: from the moment of a failure, every op fails until the bus is cleared and the
  // controller reset. Without the recovery step this stays in PROBING for ever.
  // FW 12: a wedge the RUNNING retries' recoveries cannot clear (stubborn = 2) is what reaches
  // BACKOFF; the recovery before the re-probe then heals it.
  Rig r;
  r.to_running(1000);
  r.io.wedged = true;
  r.io.stubborn = int(RUNNING_MAX_RETRIES);
  r.run_ms(10);
  CHECK(r.svc.state() == S::BACKOFF);
  const size_t at_error = r.io.log.size();
  for (int i = 0; i < 20000 && r.svc.state() != S::RUNNING; i++) { r.pass(); r.t_us += 500; }
  CHECK(r.svc.state() == S::RUNNING);
  CHECK(r.svc.chip_ok());
  CHECK(r.io.recovers == 3);              // two RUNNING retries, then the one before the re-probe
  REQUIRE(r.io.log.size() > at_error + 1);
  CHECK(r.io.log[at_error] == 'r');      // the first bus step after the error is the recovery ...
  CHECK(r.io.log[at_error + 1] == 'i');  // ... then the re-probe
  CHECK(r.io.max_pass_txns <= 1);
  CHECK(r.svc.counters().i2c_errors == 3);
  // And again: a second wedge heals the same way, no matter how many came before.
  for (int k = 0; k < 5; k++) {
    r.run_ms(12000);                     // past HEALTHY_RESET_MS, so the back-off restarts at 1 s
    r.io.wedged = true;
    r.io.stubborn = int(RUNNING_MAX_RETRIES);
    r.run_ms(10);
    CHECK(r.svc.state() == S::BACKOFF);
    for (int i = 0; i < 20000 && r.svc.state() != S::RUNNING; i++) { r.pass(); r.t_us += 500; }
    CHECK(r.svc.state() == S::RUNNING);
  }
  CHECK(r.io.recovers == 18);
}

TEST_CASE("a wedge that outlasts the first recovery is retried with a recovery before every probe") {
  Rig r;
  r.to_running(500);
  r.io.wedged = true;
  r.io.stubborn = int(RUNNING_MAX_RETRIES);
  r.run_ms(10);
  REQUIRE(r.svc.state() == S::BACKOFF);
  // The bus stays stuck through three recoveries (a stuck device would, until its byte ends).
  int left = 3;
  for (int i = 0; i < 40000 && r.svc.state() != S::RUNNING; i++) {
    const int before = r.io.recovers;
    r.pass();
    r.t_us += 500;
    if (r.io.recovers != before && left-- > 0) r.io.wedged = true;
  }
  CHECK(r.svc.state() == S::RUNNING);
  CHECK(r.io.recovers == 2 + 4);         // the two RUNNING retries, then one before each of 4 probes
  CHECK(r.io.max_pass_txns <= 1);
}

// ----------------------------------------------------------------------------------------
//  THE RUNNING RETRY POLICY (FW 12, logbook 2026-11-20-platform-imu-i2c-error-bursts)
// ----------------------------------------------------------------------------------------

TEST_CASE("RUNNING retry: a failed FIFO_COUNT is recovered and counted again, no BACKOFF, no DISCONTINUITY") {
  Rig r;
  r.to_running(500);
  r.to_drain_end();
  const size_t at = r.io.log.size();
  r.io.fail_c = 1;
  r.run_ms(500);
  CHECK(r.svc.state() == S::RUNNING);
  CHECK(r.svc.chip_ok());
  CHECK(r.svc.fail_streak() == 0);
  CHECK(r.svc.retry_streak() == 0);
  CHECK(r.svc.counters().i2c_errors == 1);    // visibility: every failure is counted
  CHECK(r.svc.counters().retries == 1);
  CHECK(r.svc.counters().recoveries == 1);
  CHECK(r.svc.counters().fifo_overflows == 0);
  REQUIRE(r.io.log.size() >= at + 4);
  CHECK(r.io.log.substr(at, 4) == "crcb");   // fail, recover, count again, burst: one step per pass
  CHECK(r.cap.appended() == r.io.consumed);   // nothing was taken from the FIFO: no sample lost
  r.freeze_and_wait();
  CHECK(r.disc_anchors() == 0);
  CHECK((g16(r.cap.header() + 18) & PlatformImuCaptureFlag::DISCONTINUITY) == 0);
}

TEST_CASE("RUNNING retry: a failed burst recovers, flushes, marks one DISCONTINUITY and stays RUNNING") {
  Rig r;
  r.to_running(500);
  r.to_drain_end();
  const uint32_t before = r.cap.appended();
  const size_t at = r.io.log.size();
  r.io.fail_b = 1;
  r.run_ms(500);
  CHECK(r.svc.state() == S::RUNNING);
  CHECK(r.svc.chip_ok());
  CHECK(r.svc.fail_streak() == 0);
  CHECK(r.svc.counters().i2c_errors == 1);
  CHECK(r.svc.counters().retries == 1);
  CHECK(r.svc.counters().recoveries == 1);
  CHECK(r.svc.counters().fifo_overflows == 0);
  REQUIRE(r.io.log.size() >= at + 5);
  CHECK(r.io.log.substr(at, 5) == "cbrfc");  // count, burst fails, recover, flush, then drains resume
  CHECK(r.cap.appended() > before + 400);     // ~0.5 s of samples after the gap
  r.freeze_and_wait();
  auto a = r.anchor_records();
  int disc = 0;
  for (size_t i = 0; i < a.size(); i++)
    if (a[i][16] & PlatformImuAnchorFlag::DISCONTINUITY) {
      disc++;
      REQUIRE(i > 0);
      CHECK(g32(a[i - 1].data()) == before);  // the closing anchor ends exactly at the gap
    }
  CHECK(disc == 1);
  CHECK(g16(r.cap.header() + 46) == 0);       // FIFO overflows in this capture
}

TEST_CASE("RUNNING retry: the third failure in a row goes to BACKOFF exactly as before") {
  SUBCASE("three failed counts") {
    Rig r;
    r.to_running(500);
    r.to_drain_end();
    const size_t at = r.io.log.size();
    r.io.fail_c = 3;
    r.run_ms(10);                            // a failure, a recovery and a retry are three 0.5 ms passes
    CHECK(r.svc.state() == S::BACKOFF);
    CHECK(r.svc.fail_streak() == 1);
    CHECK(r.svc.counters().i2c_errors == 3);
    CHECK(r.svc.counters().retries == 2);
    CHECK(r.io.log.substr(at) == "crcrc");
    const size_t at_backoff = r.io.log.size();
    for (int i = 0; i < 20000 && r.svc.state() != S::RUNNING; i++) { r.pass(); r.t_us += 500; }
    CHECK(r.svc.state() == S::RUNNING);
    REQUIRE(r.io.log.size() > at_backoff + 1);
    CHECK(r.io.log.substr(at_backoff, 2) == "ri");  // the recovery before the re-probe, as in FW 11
    r.run_ms(300);
    r.freeze_and_wait();
    CHECK(r.disc_anchors() == 1);           // the BACKOFF gap
  }
  SUBCASE("a failed burst, then two failed flushes") {
    Rig r;
    r.to_running(500);
    r.to_drain_end();
    const size_t at = r.io.log.size();
    r.io.fail_b = 1;
    r.io.fail_f = 2;
    r.run_ms(10);
    CHECK(r.svc.state() == S::BACKOFF);
    CHECK(r.svc.counters().i2c_errors == 3);
    CHECK(r.io.log.substr(at) == "cbrfrf");
  }
  SUBCASE("a failed burst, then one failed flush: the second flush succeeds and RUNNING goes on") {
    Rig r;
    r.to_running(500);
    r.to_drain_end();
    const size_t at = r.io.log.size();
    r.io.fail_b = 1;
    r.io.fail_f = 1;
    r.run_ms(500);
    CHECK(r.svc.state() == S::RUNNING);
    CHECK(r.svc.counters().i2c_errors == 2);
    CHECK(r.svc.counters().retries == 2);
    CHECK(r.io.log.substr(at, 7) == "cbrfrfc");
    r.freeze_and_wait();
    CHECK(r.disc_anchors() == 1);           // one gap: the burst; the failed flush adds none
  }
}

TEST_CASE("RUNNING retry: a successful transfer resets the run of failures") {
  Rig r;
  r.to_running(500);
  r.to_drain_end();
  const size_t at = r.io.log.size();
  r.io.fail_c = 2;                           // two failed counts, then a good one ...
  r.io.fail_b = 1;                           // ... then a failed burst: the 3rd failure, but not in a row
  r.run_ms(500);
  CHECK(r.svc.state() == S::RUNNING);
  CHECK(r.svc.fail_streak() == 0);
  CHECK(r.svc.counters().i2c_errors == 3);
  CHECK(r.svc.counters().retries == 3);
  CHECK(r.svc.counters().recoveries == 3);
  CHECK(r.svc.counters().fifo_overflows == 0);  // ~3 ms of retries, far inside the 128-packet FIFO
  CHECK(r.io.log.substr(at, 9) == "crcrcbrfc");
  // Isolated failures for a long time never reach BACKOFF (the robot's one every 15-40 s).
  for (int k = 0; k < 20; k++) {
    r.to_drain_end();
    if (k % 2) r.io.fail_c = 2; else r.io.fail_b = 2;
    r.run_ms(300);
  }
  CHECK(r.svc.state() == S::RUNNING);
  CHECK(r.svc.counters().i2c_errors == 3 + 40);
  CHECK(r.io.max_pass_txns <= 1);
}

TEST_CASE("RUNNING retry: a FwUpdate BEGIN between a failure and its recovery stops all bus work") {
  Rig r;
  r.to_running(500);
  r.to_drain_end();
  r.io.fail_c = 1;
  for (int i = 0; i < 100 && r.svc.counters().i2c_errors == 0; i++) { r.pass(); r.t_us += 500; }
  REQUIRE(r.svc.counters().i2c_errors == 1);
  REQUIRE(r.io.log.back() == 'c');           // the recovery is due on the next pass
  const size_t at = r.io.log.size();
  r.fw_open = true;
  r.run_ms(3000);
  CHECK(r.svc.state() == S::FW_HALTED);
  CHECK(r.io.log.size() == at);              // no recovery, no transfer while the session is open
  CHECK(r.io.recovers == 0);
  r.fw_open = false;                         // abandoned: PROBING, the pending recovery first
  for (int i = 0; i < 20000 && r.svc.state() != S::RUNNING; i++) { r.pass(); r.t_us += 500; }
  CHECK(r.svc.state() == S::RUNNING);
  REQUIRE(r.io.log.size() > at + 1);
  CHECK(r.io.log.substr(at, 2) == "ri");
}
