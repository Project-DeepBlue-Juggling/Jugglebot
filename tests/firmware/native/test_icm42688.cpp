// =============================================================================
//  test_icm42688.cpp - compiled test of the Platform Teensy's ICM-42688 non-blocking init
// =============================================================================
//  Links the REAL icm42688.cpp against the scripted Wire fake (hal_shims/Wire.h). Pins, on the host, what the
//  board will do on the bus: the exact register write sequence for +/-16 g, +/-1000 dps, 1 kHz; the datasheet
//  waits (10 ms after soft reset, RESET_DONE polls 1 ms apart capped at 20, 50 ms before the final flush) as
//  elapsed time on a clock that starts just before the micros() wrap; at most ONE I2C transaction per initStep
//  call; an absent chip never reports OK and never spins; an error mid-init is sticky and retryable.
//  The register sequence is also written to icm_writes.json (when IMU_DUMP_DIR is set) so
//  tests/firmware/test_platform_imu_native.py can compare it with the bench driver's source.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "Wire.h"
#include "../../../ros_ws/src/jugglebot/Teensy_code_platform/icm42688.h"

using icm42688::Driver;

namespace {

constexpr uint32_t T0 = 0xFFFFFF00u;  // 256 us before the micros() wrap

struct Rig {
  TwoWire w;
  Driver d;
  uint32_t t = T0;
  int worst_txn_per_call = 0;
  int calls = 0;
  Rig(uint8_t addr = 0x68) : d(w) { w.set_device(addr); }
  // One initStep at the current clock; asserts the one-transaction rule.
  int step() {
    int before = w.txns;
    w.now_us = t;
    int r = d.initStep(t);
    int used = w.txns - before;
    if (used > worst_txn_per_call) worst_txn_per_call = used;
    calls++;
    return r;
  }
  // Drive until not pending; `dt` us pass between calls. Returns the final status, or 12345 if it never ended.
  int run(uint32_t dt, int max_calls = 100000) {
    for (int i = 0; i < max_calls; i++) {
      int r = step();
      if (r != Driver::INIT_PENDING) return r;
      t += dt;
    }
    return 12345;
  }
  std::vector<std::pair<int, int>> writes() const {
    std::vector<std::pair<int, int>> v;
    for (const auto& e : w.log)
      if (e.kind == 'W' && !e.nacked) v.push_back({e.reg, e.val});
    return v;
  }
};

}  // namespace

TEST_CASE("init writes exactly the agreed register sequence: +/-16 g, +/-1000 dps, 1 kHz") {
  Rig r;
  r.d.initStart();
  CHECK(r.run(100) == icm42688::OK);
  auto w = r.writes();
  const std::vector<std::pair<int, int>> want = {
      {0x76, 0x00},   // BANK_SEL (probe)
      {0x76, 0x00},   // BANK_SEL
      {0x11, 0x01},   // DEVICE_CONFIG soft reset
      {0x76, 0x00},   // BANK_SEL (after the reset wait)
      {0x64, 0x00},   // INT_CONFIG1: INT_ASYNC_RESET cleared
      {0x4C, 0x70},   // INTF_CONFIG0: FIFO count in records, big-endian
      {0x4F, 0x26},   // GYRO_CONFIG0: FS_SEL 1 (1000 dps) << 5 | ODR 6 (1 kHz)
      {0x50, 0x06},   // ACCEL_CONFIG0: FS_SEL 0 (16 g) << 5 | ODR 6
      {0x52, 0x00},   // GYRO_ACCEL_CONFIG0: UI filter at the bench value (UI_FILT_CODE, knowingly unverified)
      {0x5F, 0x4F},   // FIFO_CONFIG1: accel+gyro+temp+timestamp, RESUME_PARTIAL_RD
      {0x16, 0x40},   // FIFO_CONFIG: stream
      {0x4E, 0x0F},   // PWR_MGMT0: accel + gyro low-noise
      {0x4B, 0x02}};  // SIGNAL_PATH_RESET: FIFO flush
  REQUIRE(w.size() == want.size());
  for (size_t i = 0; i < want.size(); i++) {
    INFO("write " << i);
    CHECK(w[i] == want[i]);
  }
  const unsigned ui_filt = Driver::UI_FILT_CODE;   // a copy: the static const has no out-of-class definition to bind a reference to
  CHECK(ui_filt == 0x00u);
  CHECK(r.d.odr() == 1000);
  CHECK(r.d.accelFs() == 16);
  CHECK(r.d.gyroFs() == 1000);
  CHECK(r.d.address() == 0x68);
  CHECK(r.d.whoAmI() == 0x47);
  if (const char* dir = std::getenv("IMU_DUMP_DIR")) {
    std::string path = std::string(dir) + "/icm_writes.json";
    FILE* f = std::fopen(path.c_str(), "w");
    REQUIRE(f);
    std::fprintf(f, "[");
    for (size_t i = 0; i < w.size(); i++) std::fprintf(f, "%s[%d,%d]", i ? "," : "", w[i].first, w[i].second);
    std::fprintf(f, "]\n");
    std::fclose(f);
  }
}

TEST_CASE("every initStep call issues at most one I2C transaction and init never delays") {
  Rig r;
  g_delay_calls = 0;
  unsigned long us0 = g_delay_us_total;
  r.d.initStart();
  CHECK(r.run(100) == icm42688::OK);
  CHECK(r.worst_txn_per_call == 1);
  CHECK(g_delay_calls == 0);                       // never delay(): the waits are elapsed-time checks
  CHECK(g_delay_us_total - us0 == 200u);           // only the datasheet's 200 us pause after the FIFO flush
  // Same on the slow path: RESET_DONE never reported, so all 20 polls run.
  Rig s;
  s.w.reset_done_after_reads = -1;
  s.d.initStart();
  CHECK(s.run(100) == icm42688::OK);
  CHECK(s.worst_txn_per_call == 1);
}

TEST_CASE("datasheet waits are elapsed time, across the micros() wrap") {
  Rig r;
  r.w.reset_done_after_reads = -1;  // RESET_DONE never set: all 20 polls
  r.d.initStart();
  CHECK(r.run(100) == icm42688::OK);
  uint32_t t_reset = 0, t_pwr = 0, t_flush = 0;
  std::vector<uint32_t> polls;
  for (const auto& e : r.w.log) {
    if (e.kind == 'W' && e.reg == 0x11) t_reset = e.t_us;
    if (e.kind == 'W' && e.reg == 0x4E) t_pwr = e.t_us;
    if (e.kind == 'W' && e.reg == 0x4B) t_flush = e.t_us;
    if (e.kind == 'R' && e.reg == 0x2D) polls.push_back(e.t_us);
  }
  REQUIRE(polls.size() == 20u);                                 // capped at INIT_STEP_RESET_POLLS, then go on
  CHECK(uint32_t(polls[0] - t_reset) >= 10000u);                // >= 10 ms after the soft reset
  for (size_t i = 1; i < polls.size(); i++) CHECK(uint32_t(polls[i] - polls[i - 1]) >= 1000u);  // 1 ms apart
  CHECK(uint32_t(t_flush - t_pwr) >= 50000u);                   // gyro start-up: >= 45 ms, the bench waited 50
  CHECK(t_reset < t_pwr);
  // The clock really crossed the wrap during the run.
  CHECK(uint32_t(r.t) < T0);
}

TEST_CASE("RESET_DONE reported early ends the poll early") {
  Rig r;
  r.w.reset_done_after_reads = 3;
  r.d.initStart();
  CHECK(r.run(100) == icm42688::OK);
  int polls = 0;
  for (const auto& e : r.w.log) if (e.kind == 'R' && e.reg == 0x2D) polls++;
  CHECK(polls == 3);
}

TEST_CASE("an init parked for longer than 2^31 us releases its wait at once instead of stalling") {
  Rig r;
  r.d.initStart();
  // Run until the 10 ms reset wait is pending, then jump the clock 40 minutes.
  int guard = 0;
  while (true) {
    r.step();
    r.t += 100;
    bool reset_written = false;
    for (const auto& e : r.w.log) if (e.kind == 'W' && e.reg == 0x11) reset_written = true;
    if (reset_written) break;
    REQUIRE(++guard < 100);
  }
  r.t += uint32_t(40ull * 60ull * 1000000ull);
  int before = r.w.txns;
  r.step();
  CHECK(r.w.txns == before + 1);  // the first poll happened immediately; the wait did not stall for ~35 minutes
}

TEST_CASE("a driver never started, a finished one and a failed one never report a false OK") {
  Rig r;
  CHECK(r.step() == icm42688::ERR_NOT_STARTED);
  CHECK(r.w.txns == 0);
  r.d.initStart();
  CHECK(r.run(100) == icm42688::OK);
  int txns = r.w.txns;
  CHECK(r.step() == icm42688::OK);               // finished: OK, and no further bus traffic
  CHECK(r.w.txns == txns);
  // A failed init keeps returning its error rather than OK.
  Rig f;
  f.w.nack_from_txn = 8;  // dies somewhere in the middle of the config writes
  f.d.initStart();
  int e = f.run(100);
  CHECK(e == icm42688::ERR_I2C);
  for (int i = 0; i < 5; i++) CHECK(f.step() == icm42688::ERR_I2C);
  CHECK_FALSE(f.d.initBusy());
}

TEST_CASE("absent chip: NACK forever never returns OK and never spins") {
  Rig r;
  r.w.nack_all = true;
  r.d.initStart();
  int e = r.run(100, 1000);
  CHECK(e == icm42688::ERR_NO_DEVICE);
  CHECK(r.calls <= 3);                  // one probe per address, then the error: not a loop
  CHECK(r.w.txns == 2);                 // 0x68 and 0x69, one transaction each
  for (int i = 0; i < 1000; i++) CHECK(r.step() == icm42688::ERR_NO_DEVICE);
  CHECK(r.w.txns == 2);                 // sticky error: no more traffic
  CHECK(r.worst_txn_per_call == 1);
  // A bus where nothing is present at all (not NACK-all) is the same.
  Rig p;
  p.w.present[0x68] = false;
  p.d.initStart();
  CHECK(p.run(100) == icm42688::ERR_NO_DEVICE);
}

TEST_CASE("probe: 0x69 is found after 0x68 NACKs; a forced address tries only that one; a wrong WHO_AM_I is reported") {
  Rig r(0x69);
  r.d.initStart();
  CHECK(r.run(100) == icm42688::OK);
  CHECK(r.d.address() == 0x69);
  Rig f(0x69);
  f.d.initStart(0x69);
  CHECK(f.run(100) == icm42688::OK);
  for (const auto& e : f.w.log) CHECK(e.addr == 0x69);   // 0x68 never touched
  Rig g;
  g.d.initStart(0x69);                                    // forced to an address with no chip
  CHECK(g.run(100) == icm42688::ERR_NO_DEVICE);
  CHECK(g.w.txns == 1);
  Rig bad;
  bad.w.regs[0x75] = 0x12;
  bad.d.initStart();
  CHECK(bad.run(100) == icm42688::ERR_WHO_AM_I);
  Rig illegal;
  illegal.d.initStart(0x70);                              // not a legal ICM address: nothing is tried
  CHECK(illegal.run(100) == icm42688::ERR_NO_DEVICE);
  CHECK(illegal.w.txns == 0);
}

TEST_CASE("an error mid-init returns to a retryable state; initStart retries from the probe") {
  Rig r;
  r.w.nack_from_txn = 10;
  r.d.initStart();
  CHECK(r.run(100) == icm42688::ERR_I2C);
  CHECK(r.d.initBusy() == false);
  // The bus recovers; initStart begins again at the probe and completes with the full sequence.
  r.w.nack_from_txn = -1;
  r.w.reset_log();
  r.d.initStart();
  CHECK(r.run(100) == icm42688::OK);
  CHECK(r.writes().size() == 13u);
  CHECK(r.w.log.front().kind == 'P');
}

TEST_CASE("error names") {
  CHECK(std::string(icm42688::errName(icm42688::ERR_NOT_STARTED)) == "init_not_started");
  CHECK(std::string(icm42688::errName(icm42688::OK)) == "ok");
}
