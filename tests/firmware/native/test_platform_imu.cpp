// =============================================================================
//  test_platform_imu.cpp — compiled test of the Platform IMU capture ring and protocol logic
// =============================================================================
//  Includes the Platform Teensy's own imu_capture.h / imu_proto.h. Contract:
//  ros_ws/docs/platform_imu_protocol.md. The ring here is small (injected storage) so wrap and
//  the 16-bit address space are cheap to reach. When IMU_DUMP_DIR is set, one case writes a
//  capture's payload + header to disk so tests/firmware/test_platform_imu_native.py can check
//  both CRCs against Python's zlib.crc32.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "../../../ros_ws/src/jugglebot/Teensy_code_platform/imu_proto.h"

using namespace ImuCapture;
using namespace ImuProto;
namespace Op = PlatformImuOp;
namespace St = PlatformImuStatus;
namespace S = PlatformImuState;

namespace {

uint32_t get32(const uint8_t* p) { return uint32_t(get16(p)) | (uint32_t(get16(p + 2)) << 16); }
bool flag16(const uint8_t* p, uint16_t m) { return (get16(p) & m) != 0; }

constexpr uint32_t CAP = 100, ACAP = 8;

struct Rig {
  int16_t buf[CAP * 6];
  Anchor anchors[ACAP];
  Capture cap;
  Engine eng;
  HeaderInfo info;
  uint32_t t_us = 1000;
  Rig(uint32_t boot = 0x12340042u) {
    cap.init(buf, CAP, anchors, ACAP, boot);
    eng.init(&cap);
  }
  void push(int n, int base = 0) {
    for (int i = 0; i < n; i++) {
      int16_t s[6];
      for (int j = 0; j < 6; j++) s[j] = int16_t((base + i) * 7 + j);
      REQUIRE(cap.push(s));
    }
  }
  Reply send(uint8_t op, uint8_t a1 = 0, uint8_t a2 = 0, uint8_t a3 = 0, uint8_t a4 = 0, uint8_t a5 = 0,
             uint8_t a6 = 0, uint8_t a7 = 0, bool* deferred = nullptr) {
    uint8_t d[8] = {op, a1, a2, a3, a4, a5, a6, a7};
    bool def;
    Reply r = eng.handle(parse_cmd(d, 8), info, deferred ? deferred : &def);
    return r;
  }
  Reply freeze(uint8_t reason = 1) {
    bool def = false;
    Reply r = send(Op::FREEZE, reason, 0, 0, 0, 0, 0, 0, &def);
    if (!def) return r;
    Reply out;
    int guard = 0;
    while (!eng.service_freeze(&out)) REQUIRE(++guard < 10000);
    return out;
  }
  Reply read(uint16_t id, uint16_t first, uint16_t count, uint8_t gap = 100) {
    return send(Op::READ, uint8_t(id), uint8_t(id >> 8), uint8_t(first), uint8_t(first >> 8), uint8_t(count),
                uint8_t(count >> 8), gap);
  }
  Reply resume(uint16_t id) { return send(Op::RESUME, uint8_t(id), uint8_t(id >> 8)); }
};

void go_running(Rig& r) {
  r.eng.on_chip_ready();
  REQUIRE(r.eng.state() == S::RUNNING);
}

}  // namespace

TEST_CASE("crc32 matches the standard check value") {
  const char* s = "123456789";
  CHECK(crc32(reinterpret_cast<const uint8_t*>(s), 9) == 0xCBF43926u);
  CHECK(crc32(nullptr, 0) == 0u);
}

TEST_CASE("ring: fill, wrap and the oldest-first order of a frozen capture") {
  Rig r;
  go_running(r);
  r.push(CAP + 30);  // wrapped: sample index 30 is the oldest retained
  CHECK(r.cap.wrapped());
  CHECK(r.cap.n_samples() == CAP);
  Reply f = r.freeze();
  CHECK(f.status == St::OK);
  CHECK(f.detail == CAP);
  uint8_t w[6];
  REQUIRE(r.cap.read_word(0, w));
  CHECK(int16_t(get16(w)) == int16_t(30 * 7 + 0));
  REQUIRE(r.cap.read_word(1, w));  // gyro half of sample 0
  CHECK(int16_t(get16(w)) == int16_t(30 * 7 + 3));
  REQUIRE(r.cap.read_word(2 * CAP - 1, w));
  CHECK(int16_t(get16(w + 4)) == int16_t((CAP + 29) * 7 + 5));
  const uint8_t* h = r.cap.header();
  CHECK(flag16(h + 18, PlatformImuCaptureFlag::WRAPPED));
  CHECK(h[0] == PlatformImu::HEADER_FORMAT_VERSION);
}

TEST_CASE("freeze makes the image stable: pushes are refused, reads do not change") {
  Rig r;
  go_running(r);
  r.push(10);
  r.freeze();
  uint8_t before[6], after[6];
  REQUIRE(r.cap.read_word(3, before));
  int16_t s[6] = {1, 2, 3, 4, 5, 6};
  CHECK_FALSE(r.cap.push(s));
  CHECK_FALSE(r.cap.add_anchor(1, 2, 0));
  REQUIRE(r.cap.read_word(3, after));
  CHECK(std::memcmp(before, after, 6) == 0);
}

TEST_CASE("resume starts a new capture and refuses a stale id") {
  Rig r;
  go_running(r);
  r.push(10);
  Reply f = r.freeze();
  uint16_t id = f.capture_id;
  CHECK(id != 0);
  CHECK(r.resume(uint16_t(id + 1)).status == St::ERR_WRONG_CAPTURE);
  CHECK(r.eng.state() == S::FROZEN);
  Reply ok = r.resume(id);
  CHECK(ok.status == St::OK);
  CHECK(ok.capture_id == id);
  CHECK(r.eng.state() == S::RUNNING);
  CHECK(r.eng.take_resume_check());
  CHECK(r.cap.n_samples() == 0);
  CHECK(r.resume(id).status == St::ERR_NOT_FROZEN);  // a retried RESUME can never thaw anything
  r.push(5);
  Reply f2 = r.freeze();
  CHECK(f2.capture_id != id);
  CHECK(f2.capture_id != 0);
}

TEST_CASE("capture_id steps, skips 0 and starts from the boot id's low 16 bits") {
  Rig r(0xABCD0000u);  // low 16 bits 0 -> starts at 1
  go_running(r);
  r.push(3);
  CHECK(r.freeze().capture_id == 1);
  Rig q(0x0000FFFFu);
  go_running(q);
  q.push(3);
  uint16_t a = q.freeze().capture_id;
  CHECK(a == 0xFFFF);
  q.resume(a);
  q.push(3);
  CHECK(q.freeze().capture_id == 1);  // wrapped past 0
}

TEST_CASE("seq address space: sample, anchor and header regions and their edges") {
  Rig r;
  go_running(r);
  r.push(20);
  r.cap.add_anchor(111, 222, 0);
  r.push(20, 20);
  r.cap.add_anchor(333, 444, PlatformImuAnchorFlag::DISCONTINUITY);
  Reply f = r.freeze();
  uint16_t id = f.capture_id;
  uint32_t N = 40, A = 2, AB = PlatformImu::ANCHOR_SEQ_BASE, HB = PlatformImu::HEADER_SEQ_BASE;
  uint8_t w[6];
  CHECK(r.cap.read_word(2 * N - 1, w));
  CHECK_FALSE(r.cap.read_word(2 * N, w));
  CHECK_FALSE(r.cap.read_word(AB - 1, w));
  CHECK(r.cap.read_word(AB, w));
  CHECK(r.cap.read_word(AB + 3 * A - 1, w));
  CHECK_FALSE(r.cap.read_word(AB + 3 * A, w));
  CHECK(r.cap.read_word(HB, w));
  CHECK(r.cap.read_word(HB + 10, w));
  CHECK_FALSE(r.cap.read_word(HB + 11, w));
  // READ legality: whole inside one region
  CHECK(r.read(id, 0, 2 * N).status == St::OK);
  r.eng.on_fw_update_begin();  // reset the engine out of UPLOADING cheaply for the next cases
  Rig q;
  go_running(q);
  q.push(20); q.cap.add_anchor(111, 222, 0); q.push(20, 20);
  uint16_t qid = q.freeze().capture_id;
  Reply bad = q.read(qid, 2 * N - 1, 2);  // straddles the sample region end
  CHECK(bad.status == St::ERR_OUT_OF_RANGE);
  CHECK(bad.detail == (uint32_t(2 * N - 1) | (uint32_t(2 * N) << 16)));
  CHECK(q.read(qid, 2 * N, 1).status == St::ERR_OUT_OF_RANGE);
  CHECK(q.read(qid, uint16_t(AB), 4).status == St::ERR_OUT_OF_RANGE);   // 4 words, only 3 valid
  CHECK(q.read(qid, uint16_t(HB), 12).status == St::ERR_OUT_OF_RANGE);  // 12 words, only 11
  CHECK(q.read(qid, uint16_t(AB - 1), 2).status == St::ERR_OUT_OF_RANGE);
  CHECK(q.eng.state() == S::FROZEN);
  CHECK_FALSE(q.eng.block_active());  // a refused READ sends nothing
  Reply ok = q.read(qid, uint16_t(HB), 11);
  CHECK(ok.status == St::OK);
  CHECK(ok.detail == (uint32_t(HB) | (11u << 16)));
}

TEST_CASE("anchors: records are 18 bytes in 3 words, oldest first, discontinuity flag survives") {
  Rig r;
  go_running(r);
  r.push(10);
  r.cap.add_anchor(1000, 5000000000ull, PlatformImuAnchorFlag::SYNC_STEP);
  r.push(10);
  r.cap.add_anchor(2000, 6000000000ull, PlatformImuAnchorFlag::DISCONTINUITY | PlatformImuAnchorFlag::WALL_INVALID);
  r.freeze();
  uint8_t rec[18];
  for (int k = 0; k < 3; k++) REQUIRE(r.cap.read_word(PlatformImu::ANCHOR_SEQ_BASE + 3 + k, rec + 6 * k));
  CHECK(get32(rec) == 20u);
  CHECK(get32(rec + 4) == 2000u);
  CHECK(rec[8] == (6000000000ull & 0xFF));
  CHECK(rec[16] == uint8_t(PlatformImuAnchorFlag::DISCONTINUITY | PlatformImuAnchorFlag::WALL_INVALID));
  CHECK(rec[17] == 0);
  CHECK(get16(r.cap.header() + 16) == 2);
  CHECK(flag16(r.cap.header() + 18, PlatformImuCaptureFlag::DISCONTINUITY));
}

TEST_CASE("anchors older than the oldest retained sample are dropped at freeze; the anchor ring overwrites") {
  Rig r;
  go_running(r);
  for (int i = 0; i < 40; i++) {  // 40 anchors, 8 slots, 100-sample ring, 10 samples apart
    r.push(10, i * 10);
    r.cap.add_anchor(uint32_t(i), uint64_t(i), 0);
  }
  r.freeze();
  // appended 400, retained 100, first_index 300; anchor ring keeps the last 8: indexes 330..400 -> all retained
  CHECK(r.cap.n_anchors() == 8);
  Rig q;
  go_running(q);
  q.push(100);
  q.cap.add_anchor(0, 0, 0);       // index 100
  q.push(100, 100);                 // wraps; first_index 100 -> the anchor at index 100 refers to sample 99: dropped
  q.cap.add_anchor(1, 1, 0);       // index 200
  q.freeze();
  CHECK(q.cap.n_anchors() == 1);
}

TEST_CASE("payload crc is chunkable and equals the one-shot crc of the words in seq order") {
  Rig r;
  go_running(r);
  r.push(CAP + 17);
  r.cap.add_anchor(5, 6, 0);
  REQUIRE(r.cap.begin_freeze(1));
  int steps = 0;
  while (!r.cap.crc_step(30)) steps++;  // 5 words per slice
  CHECK(steps > 10);
  std::vector<uint8_t> all;
  uint8_t w[6];
  for (uint32_t p = 0; p < r.cap.payload_words(); p++) {
    uint32_t seq = p < 3u * r.cap.n_anchors() ? PlatformImu::ANCHOR_SEQ_BASE + p : p - 3u * r.cap.n_anchors();
    REQUIRE(r.cap.read_word(seq, w));
    all.insert(all.end(), w, w + 6);
  }
  CHECK(r.cap.payload_crc() == crc32(all.data(), all.size()));
  CHECK_FALSE(r.cap.header_ready());
  uint8_t hw[6];
  CHECK_FALSE(r.cap.read_word(PlatformImu::HEADER_SEQ_BASE, hw));  // not readable before finalize
  CHECK(r.cap.finalize(HeaderInfo{}));
  CHECK(r.cap.read_word(PlatformImu::HEADER_SEQ_BASE, hw));
  const uint8_t* h = r.cap.header();
  uint32_t pc = uint32_t(get16(h + 54)) | (uint32_t(get16(h + 56)) << 16);
  uint32_t hc = uint32_t(get16(h + 62)) | (uint32_t(get16(h + 64)) << 16);
  CHECK(pc == r.cap.payload_crc());
  CHECK(hc == crc32(h, 62));
}

TEST_CASE("header fields land at the spec offsets") {
  Rig r(0x11223344u);
  go_running(r);
  r.push(5);
  HeaderInfo hi;
  hi.freeze_wall_us = 0x0102030405060708ull; hi.freeze_local_us = 0xAABBCCDDu; hi.odr_hz = 1000;
  hi.accel_lsb_per_g = 2048; hi.gyro_lsb_per_dps_x10 = 328; hi.reg_gyro_accel_config0 = 0x11;
  hi.reg_accel_config0 = 0x26; hi.reg_gyro_config0 = 0x26; hi.i2c_clock_100khz = 4;
  hi.temp_start_raw = -5; hi.temp_freeze_raw = 300; hi.fifo_overflows = 7; hi.i2c_errors = 9;
  hi.sync_peak_us = 1234; hi.fw_version = 11; hi.extra_flags = PlatformImuCaptureFlag::NEVER_SYNCED;
  r.info = hi;
  r.freeze(PlatformImuFreezeReason::FAULT);
  const uint8_t* h = r.cap.header();
  CHECK(h[1] == PlatformImuFreezeReason::FAULT);
  CHECK(get16(h + 4) == 0x3344);
  CHECK(get32(h + 8) == 5);
  CHECK(get32(h + 12) == 0);
  CHECK(h[20] == 0x08);
  CHECK(h[27] == 0x01);
  CHECK(get16(h + 28) == 0xCCDD);
  CHECK(get16(h + 32) == 1000);
  CHECK(get16(h + 34) == 2048);
  CHECK(get16(h + 36) == 328);
  CHECK(h[38] == 0x11);
  CHECK(h[39] == 0x26);
  CHECK(h[40] == 0x26);
  CHECK(h[41] == 4);
  CHECK(int16_t(get16(h + 42)) == -5);
  CHECK(get16(h + 44) == 300);
  CHECK(get16(h + 46) == 7);
  CHECK(get16(h + 48) == 9);
  CHECK(get16(h + 50) == 1234);
  CHECK(get16(h + 52) == 11);
  CHECK(get16(h + 18) == PlatformImuCaptureFlag::NEVER_SYNCED);
  CHECK(get16(h + 58) == 0);
  CHECK(get16(h + 60) == 0);
}

TEST_CASE("frame pack and unpack round trips") {
  Reply r;
  r.op = Op::READ; r.status = St::ERR_OUT_OF_RANGE; r.capture_id = 0x4A21; r.detail = 0xFFF0u | (11u << 16);
  Reply u = unpack_reply(pack_reply(r));
  CHECK(u.op == r.op);
  CHECK(u.status == r.status);
  CHECK(u.capture_id == r.capture_id);
  CHECK(u.detail == r.detail);
  Frame f = pack_reply(Reply{Op::FREEZE, 0, 0x4A21, 32000});
  const uint8_t ex[8] = {0x03, 0x00, 0x21, 0x4A, 0x00, 0x7D, 0x00, 0x00};  // the spec walk-through [03][00][0x4A21][32000]
  CHECK(std::memcmp(f.d, ex, 8) == 0);
  Status s;
  s.state = S::RUNNING; s.flags = 0x0B; s.temp_c = -3; s.fifo_overflows = 1; s.i2c_errors = 2;
  s.sync_peak = 254; s.fill_s = 31; s.boot = 0x42;
  Frame sf = pack_status(s);
  CHECK(sf.d[0] == 2); CHECK(sf.d[1] == 0x0B); CHECK(int8_t(sf.d[2]) == -3); CHECK(sf.d[3] == 1);
  CHECK(sf.d[4] == 2); CHECK(sf.d[5] == 254); CHECK(sf.d[6] == 31); CHECK(sf.d[7] == 0x42);
  const uint8_t w6[6] = {1, 2, 3, 4, 5, 6};
  Frame df = pack_data(0xFFF3, w6);
  CHECK(df.d[0] == 0xF3); CHECK(df.d[1] == 0xFF); CHECK(std::memcmp(df.d + 2, w6, 6) == 0);
  CHECK(fill_seconds(32000, 1000) == 32);
  CHECK(fill_seconds(300000, 1000) == 255);
  const uint8_t rd[8] = {Op::READ, 0x21, 0x4A, 0x35, 0x12, 0x01, 0x00, 100};
  Cmd c = parse_cmd(rd, 8);
  CHECK(c.valid); CHECK(c.capture_id == 0x4A21); CHECK(c.first_seq == 0x1235); CHECK(c.count == 1); CHECK(c.gap_10us == 100);
}

TEST_CASE("parse_cmd: bad ops, short frames and argument ranges are invalid") {
  uint8_t d[8] = {0};
  CHECK_FALSE(parse_cmd(d, 0).valid);
  d[0] = 0x00; CHECK_FALSE(parse_cmd(d, 8).valid);
  d[0] = 0x07; CHECK_FALSE(parse_cmd(d, 8).valid);
  d[0] = Op::FREEZE; d[1] = 0; CHECK_FALSE(parse_cmd(d, 8).valid);  // reason 0 is ERR_BAD_ARGS
  d[1] = 5; CHECK_FALSE(parse_cmd(d, 8).valid);
  d[1] = 4; CHECK(parse_cmd(d, 8).valid);
  CHECK_FALSE(parse_cmd(d, 1).valid);
  d[0] = Op::READ; d[5] = 0; d[6] = 0; CHECK_FALSE(parse_cmd(d, 8).valid);  // count 0
}

TEST_CASE("parse_cmd READ count edges: 1 and 256 pass, 0 and 257 fail") {
  uint8_t d[8] = {Op::READ, 1, 0, 0, 0, 0, 0, 100};
  auto at = [&](uint16_t n) { d[5] = uint8_t(n); d[6] = uint8_t(n >> 8); return parse_cmd(d, 8).valid; };
  CHECK_FALSE(at(0));
  CHECK(at(1));
  CHECK(at(256));
  CHECK_FALSE(at(257));
  CHECK_FALSE(parse_cmd(d, 7).valid);
}

TEST_CASE("state machine legality table") {
  // Build one rig per state and drive every op, comparing against the spec's table.
  struct Row { uint8_t op; uint8_t status; uint8_t next; };
  auto in_state = [](uint8_t target, Rig& r) {
    switch (target) {
      case S::DISABLED: r.send(Op::DISABLE); break;
      case S::PROBING: break;
      case S::RUNNING: r.push(3); r.eng.on_chip_ready(); break;
      case S::BACKOFF: r.push(3); r.eng.on_chip_ready(); r.eng.on_i2c_error(0); break;
      case S::FROZEN: r.push(3); r.eng.on_chip_ready(); r.freeze(); break;
      case S::UPLOADING: {
        r.push(3); r.eng.on_chip_ready();
        uint16_t id = r.freeze().capture_id;
        REQUIRE(r.read(id, 0, 6).status == St::OK);
        break;
      }
      case S::FW_HALTED: r.eng.on_fw_update_begin(); break;
    }
    REQUIRE(r.eng.state() == target);
  };
  auto id_of = [](Rig& r) { return r.cap.capture_id(); };

  for (uint8_t st : {S::DISABLED, S::PROBING, S::RUNNING, S::BACKOFF, S::FROZEN, S::UPLOADING, S::FW_HALTED}) {
    // PROBING and BACKOFF with samples: FREEZE is allowed with SENSOR_ERROR; RESUME/READ ERR_NOT_FROZEN.
    for (uint8_t op : {Op::ENABLE, Op::DISABLE, Op::FREEZE, Op::RESUME, Op::READ, Op::ABORT}) {
      Rig r;
      if (st == S::PROBING) r.push(3);
      in_state(st, r);
      uint16_t id = id_of(r);
      Reply rep;
      bool def = false;
      uint8_t d[8] = {op, 1, uint8_t(id), uint8_t(id >> 8), 0, 0, 0, 0};
      if (op == Op::RESUME) { d[1] = uint8_t(id); d[2] = uint8_t(id >> 8); }
      if (op == Op::READ) { d[1] = uint8_t(id); d[2] = uint8_t(id >> 8); d[3] = 0; d[4] = 0; d[5] = 2; d[6] = 0; d[7] = 100; }
      rep = r.eng.handle(parse_cmd(d, 8), r.info, &def);
      INFO("state " << int(st) << " op " << int(op));
      uint8_t want_status = St::OK, want_next = st;
      switch (st) {
        case S::DISABLED:
          if (op == Op::ENABLE) want_next = S::PROBING;
          if (op == Op::FREEZE || op == Op::RESUME || op == Op::READ) want_status = St::ERR_DISABLED;
          break;
        case S::PROBING: case S::BACKOFF:
          if (op == Op::DISABLE) want_next = S::DISABLED;
          if (op == Op::FREEZE) want_next = S::FROZEN;
          if (op == Op::RESUME || op == Op::READ) want_status = St::ERR_NOT_FROZEN;
          break;
        case S::RUNNING:
          if (op == Op::DISABLE) want_next = S::DISABLED;
          if (op == Op::FREEZE) want_next = S::FROZEN;
          if (op == Op::RESUME || op == Op::READ) want_status = St::ERR_NOT_FROZEN;
          break;
        case S::FROZEN:
          if (op == Op::DISABLE) want_next = S::DISABLED;
          if (op == Op::FREEZE) want_status = St::ALREADY_FROZEN;
          if (op == Op::RESUME) want_next = S::RUNNING;
          if (op == Op::READ) want_next = S::UPLOADING;
          break;
        case S::UPLOADING:
          if (op == Op::DISABLE) want_next = S::DISABLED;
          if (op == Op::FREEZE) want_status = St::ALREADY_FROZEN;
          if (op == Op::RESUME) want_next = S::RUNNING;
          if (op == Op::READ) want_next = S::UPLOADING;
          if (op == Op::ABORT) want_next = S::FROZEN;
          break;
        case S::FW_HALTED:
          want_status = St::ERR_FW_UPDATE;
          break;
      }
      if (st == S::FROZEN || st == S::UPLOADING) {} // ids are real
      if ((st == S::PROBING || st == S::BACKOFF || st == S::RUNNING) && op == Op::FREEZE) {
        CHECK(def);  // accepted, reply comes from service_freeze
      } else {
        CHECK(rep.status == want_status);
      }
      CHECK(r.eng.state() == want_next);
      CHECK(rep.op == op);
    }
  }
}

TEST_CASE("freeze with no samples is ERR_EMPTY; a sensor-less freeze records SENSOR_ERROR") {
  Rig r;
  CHECK(r.send(Op::FREEZE, 1).status == St::ERR_EMPTY);
  CHECK(r.eng.state() == S::PROBING);
  r.push(4);
  Reply f = r.freeze();  // still PROBING
  CHECK(f.status == St::OK);
  CHECK(r.eng.state() == S::FROZEN);
  CHECK(flag16(r.cap.header() + 18, PlatformImuCaptureFlag::SENSOR_ERROR));
}

TEST_CASE("a second FREEZE answers ALREADY_FROZEN with the existing id and n") {
  Rig r;
  go_running(r);
  r.push(9);
  uint16_t id = r.freeze().capture_id;
  Reply again = r.send(Op::FREEZE, 2);
  CHECK(again.status == St::ALREADY_FROZEN);
  CHECK(again.capture_id == id);
  CHECK(again.detail == 9);
}

TEST_CASE("capture_id mismatch refusals change nothing") {
  Rig r;
  go_running(r);
  r.push(9);
  uint16_t id = r.freeze().capture_id;
  Reply a = r.read(uint16_t(id ^ 1), 0, 2);
  CHECK(a.status == St::ERR_WRONG_CAPTURE);
  CHECK(r.eng.state() == S::FROZEN);
  CHECK_FALSE(r.eng.block_active());
  Reply b = r.resume(0);
  CHECK(b.status == St::ERR_WRONG_CAPTURE);
  CHECK(r.eng.state() == S::FROZEN);
  CHECK(r.cap.n_samples() == 9);
}

TEST_CASE("READ block pacing: gap floor, wrap-safe clock, uncommitted frames do not advance") {
  CHECK(Engine::gap_for(100) == 1000u);
  CHECK(Engine::gap_for(0) == PlatformImu::READ_MIN_GAP_10US * 10u);
  CHECK(Engine::gap_for(39) == PlatformImu::READ_MIN_GAP_10US * 10u);
  CHECK(Engine::gap_for(255) == 2550u);
  Rig r;
  go_running(r);
  r.push(10);
  uint16_t id = r.freeze().capture_id;
  Reply ok = r.read(id, 0, 3, 100);
  REQUIRE(ok.status == St::OK);
  CHECK(r.eng.state() == S::UPLOADING);
  uint32_t t = 0xFFFFFF00u;  // near the 32-bit wrap
  Frame f{};
  REQUIRE(r.eng.peek_data(t, &f));  // first frame is due at once
  CHECK(get16(f.d) == 0);
  REQUIRE(r.eng.peek_data(t, &f));  // peek does not advance: a mailbox that was busy retries the same frame
  CHECK(get16(f.d) == 0);
  r.eng.commit_data(t);
  CHECK_FALSE(r.eng.peek_data(t + 999, &f));
  CHECK(r.eng.data_wait_us(t + 999) == 1);
  REQUIRE(r.eng.peek_data(t + 1000, &f));  // across the wrap
  CHECK(get16(f.d) == 1);
  r.eng.commit_data(t + 1000);
  REQUIRE(r.eng.peek_data(t + 2000, &f));
  CHECK(get16(f.d) == 2);
  r.eng.commit_data(t + 2000);
  CHECK(r.eng.state() == S::FROZEN);  // last frame handed to FlexCAN: UPLOADING -> FROZEN
  CHECK_FALSE(r.eng.peek_data(t + 9999, &f));
}

TEST_CASE("a data frame carries exactly the word its seq names") {
  Rig r;
  go_running(r);
  r.push(10);
  uint16_t id = r.freeze().capture_id;
  REQUIRE(r.read(id, 4, 1).status == St::OK);
  Frame f{};
  REQUIRE(r.eng.peek_data(0, &f));
  uint8_t w[6];
  REQUIRE(r.cap.read_word(4, w));
  CHECK(get16(f.d) == 4);
  CHECK(std::memcmp(f.d + 2, w, 6) == 0);
}

TEST_CASE("ABORT cuts a block and a replacing READ restarts it; RESUME cuts it too") {
  Rig r;
  go_running(r);
  r.push(10);
  uint16_t id = r.freeze().capture_id;
  REQUIRE(r.read(id, 0, 6).status == St::OK);
  CHECK(r.eng.block_active());
  CHECK(r.send(Op::ABORT).status == St::OK);
  CHECK(r.eng.state() == S::FROZEN);
  CHECK_FALSE(r.eng.block_active());
  REQUIRE(r.read(id, 0, 6).status == St::OK);
  REQUIRE(r.read(id, 8, 2).status == St::OK);  // replaces the block
  Frame f{};
  REQUIRE(r.eng.peek_data(0, &f));
  CHECK(get16(f.d) == 8);
  CHECK(r.resume(id).status == St::OK);
  CHECK_FALSE(r.eng.block_active());
  CHECK_FALSE(r.eng.peek_data(0, &f));
}

TEST_CASE("internal transitions: probing, backoff, firmware update") {
  Rig r;
  CHECK(r.eng.state() == S::PROBING);
  CHECK_FALSE(r.eng.sampling());
  r.eng.on_chip_ready();
  CHECK(r.eng.state() == S::RUNNING);
  CHECK(r.eng.sampling());
  r.eng.on_i2c_error(5000);
  CHECK(r.eng.state() == S::BACKOFF);
  CHECK(r.eng.take_discontinuity());
  CHECK_FALSE(r.eng.take_discontinuity());
  r.eng.tick(5999);
  CHECK(r.eng.state() == S::BACKOFF);
  r.eng.tick(6000);
  CHECK(r.eng.state() == S::PROBING);
  r.eng.on_chip_ready();
  r.push(3);
  r.eng.on_resume_chip_failed();  // only meaningful straight after a RESUME
  CHECK(r.eng.state() == S::PROBING);
  r.eng.on_fw_update_begin();
  CHECK(r.eng.state() == S::FW_HALTED);
  CHECK(r.cap.n_samples() == 0);  // capture discarded
  r.eng.on_chip_ready();
  CHECK(r.eng.state() == S::FW_HALTED);
  r.eng.on_fw_update_abandoned();
  CHECK(r.eng.state() == S::PROBING);
  // I2C errors outside RUNNING change nothing
  r.eng.on_i2c_error(0);
  CHECK(r.eng.state() == S::PROBING);
}

TEST_CASE("backoff keeps the ring: recovery appends to the same capture") {
  Rig r;
  go_running(r);
  r.push(5);
  r.eng.on_i2c_error(0);
  r.eng.tick(1000);
  r.eng.on_chip_ready();
  r.push(5);
  CHECK(r.cap.n_samples() == 10);
}

TEST_CASE("ENABLE from DISABLED starts a fresh capture") {
  Rig r;
  go_running(r);
  r.push(5);
  r.send(Op::DISABLE);
  CHECK(r.send(Op::ENABLE).status == St::OK);
  CHECK(r.eng.state() == S::PROBING);
  CHECK(r.cap.n_samples() == 0);
}

TEST_CASE("status cadence: due at once, then once per period, wrap-safe") {
  Rig r;
  CHECK(r.eng.status_due(100));
  CHECK_FALSE(r.eng.status_due(100 + PlatformImu::STATUS_PERIOD_MS - 1));
  CHECK(r.eng.status_due(100 + PlatformImu::STATUS_PERIOD_MS));
  Rig q;
  CHECK(q.eng.status_due(0xFFFFFF00u));
  CHECK(q.eng.status_due(0xFFFFFF00u + PlatformImu::STATUS_PERIOD_MS));  // crosses 2^32
}

TEST_CASE("a freeze reply is sent once, and not before the CRC is done") {
  Rig r;
  go_running(r);
  r.push(CAP);
  bool def = false;
  Reply held = r.send(Op::FREEZE, 1, 0, 0, 0, 0, 0, 0, &def);
  CHECK(def);
  CHECK(r.eng.freeze_pending());
  Reply out;
  while (!r.eng.service_freeze(&out)) {}
  CHECK(out.status == St::OK);
  CHECK(out.detail == CAP);
  CHECK_FALSE(r.eng.freeze_pending());
  CHECK_FALSE(r.eng.service_freeze(&out));
  (void)held;
}

TEST_CASE("full-size address space fits the 16-bit seq") {
  // the real regions: 2N <= 64000 < anchor base; 3 x anchor_ring ends below the header base; header ends below 65536
  CHECK(2u * PlatformImu::RING_SAMPLES <= PlatformImu::ANCHOR_SEQ_BASE);
  CHECK(PlatformImu::ANCHOR_SEQ_BASE + 3u * PlatformImu::ANCHOR_RING <= PlatformImu::HEADER_SEQ_BASE);
  CHECK(PlatformImu::HEADER_SEQ_BASE + PlatformImu::HEADER_BYTES / PlatformImu::WORD_BYTES <= 65536u);
  CHECK(PlatformImu::HEADER_BYTES % PlatformImu::WORD_BYTES == 0);
  CHECK(PlatformImu::BYTES_PER_SAMPLE == 12u);
}

TEST_CASE("dump a wrapped capture for the Python zlib cross-check") {
  const char* dir = std::getenv("IMU_DUMP_DIR");
  if (!dir) return;
  Rig r(0xCAFE0007u);
  go_running(r);
  for (int i = 0; i < 250; i++) {
    r.push(1, i * 13 - 900);
    if (i % 25 == 24) r.cap.add_anchor(uint32_t(i * 1000), 1700000000000000ull + uint64_t(i) * 1000,
                                       i == 199 ? PlatformImuAnchorFlag::DISCONTINUITY : 0);
  }
  r.info.fw_version = 11;
  Reply f = r.freeze();
  REQUIRE(f.status == St::OK);
  std::vector<uint8_t> out;
  uint8_t w[6];
  uint32_t A = r.cap.n_anchors(), N = r.cap.n_samples();
  for (uint32_t p = 0; p < 3 * A; p++) { REQUIRE(r.cap.read_word(PlatformImu::ANCHOR_SEQ_BASE + p, w)); out.insert(out.end(), w, w + 6); }
  for (uint32_t p = 0; p < 2 * N; p++) { REQUIRE(r.cap.read_word(p, w)); out.insert(out.end(), w, w + 6); }
  for (uint32_t p = 0; p < 11; p++) { REQUIRE(r.cap.read_word(PlatformImu::HEADER_SEQ_BASE + p, w)); out.insert(out.end(), w, w + 6); }
  std::string path = std::string(dir) + "/capture.bin";
  FILE* fp = std::fopen(path.c_str(), "wb");
  REQUIRE(fp != nullptr);
  std::fwrite(out.data(), 1, out.size(), fp);
  std::fclose(fp);
}
