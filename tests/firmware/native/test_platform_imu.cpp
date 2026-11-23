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
  hi.reg_accel_config0 = 0x06; hi.reg_gyro_config0 = 0x26; hi.i2c_clock_100khz = 4;
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
  CHECK(h[39] == 0x06);
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
      // Spec: every reply carries the frozen capture's id (0 when none); DISABLE discards it, so 0.
      if (!def) CHECK(rep.capture_id == (((st == S::FROZEN || st == S::UPLOADING) && op != Op::DISABLE) ? id : 0));
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
  // Every pass-through header field gets a distinct value, so a transposed pair of fields fails in Python.
  r.info.freeze_wall_us = 0x1122334455667788ull; r.info.freeze_local_us = 0x99AABBCCu;
  r.info.odr_hz = 1000; r.info.accel_lsb_per_g = 2048; r.info.gyro_lsb_per_dps_x10 = 328;
  r.info.reg_gyro_accel_config0 = 0x31; r.info.reg_accel_config0 = 0x06; r.info.reg_gyro_config0 = 0x26;
  r.info.i2c_clock_100khz = 4; r.info.temp_start_raw = -321; r.info.temp_freeze_raw = 4321;
  r.info.fifo_overflows = 17; r.info.i2c_errors = 29; r.info.sync_peak_us = 1357; r.info.fw_version = 11;
  r.info.sync_rx_delay_10us = 2468; r.info.rx_ring_depth_max = 37; r.info.rx_ring_leak_max = 5;   // FW 15
  r.info.extra_flags = PlatformImuCaptureFlag::NEVER_SYNCED | PlatformImuCaptureFlag::I2C_400K;
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
  // The fed values, for the independent struct-based layout check in Python.
  std::string jp = std::string(dir) + "/header_info.json";
  FILE* jf = std::fopen(jp.c_str(), "w");
  REQUIRE(jf != nullptr);
  const HeaderInfo& i = r.info;
  std::fprintf(jf,
               "{\"boot_id\":%u,\"reason\":1,\"freeze_wall_us\":%llu,\"freeze_local_us\":%u,\"odr_hz\":%u,"
               "\"accel_lsb_per_g\":%u,\"gyro_lsb_per_dps_x10\":%u,\"reg_gyro_accel_config0\":%u,"
               "\"reg_accel_config0\":%u,\"reg_gyro_config0\":%u,\"i2c_clock_100khz\":%u,\"temp_start_raw\":%d,"
               "\"temp_freeze_raw\":%d,\"fifo_overflows\":%u,\"i2c_errors\":%u,\"sync_peak_us\":%u,"
               "\"fw_version\":%u,\"extra_flags\":%u,\"sync_rx_delay_10us\":%u,\"rx_ring_depth_max\":%u,"
               "\"rx_ring_leak_max\":%u}\n",
               0xCAFE0007u, (unsigned long long)i.freeze_wall_us, i.freeze_local_us, i.odr_hz, i.accel_lsb_per_g,
               i.gyro_lsb_per_dps_x10, i.reg_gyro_accel_config0, i.reg_accel_config0, i.reg_gyro_config0,
               i.i2c_clock_100khz, i.temp_start_raw, i.temp_freeze_raw, i.fifo_overflows, i.i2c_errors,
               i.sync_peak_us, i.fw_version, i.extra_flags, i.sync_rx_delay_10us, i.rx_ring_depth_max,
               i.rx_ring_leak_max);
  std::fclose(jf);
}


/* ===================== P1 audit follow-up: wrap, resume gating, ids, pacing, real sizes ===================== */

namespace {

// A ring of any (small) size with its own storage, for the anchor-ring and wrap cases.
struct SmallRig {
  std::vector<int16_t> buf;
  std::vector<Anchor> anchors;
  Capture cap;
  HeaderInfo info;
  SmallRig(uint32_t n, uint32_t an, uint32_t boot = 0x5150u) : buf(n * 6), anchors(an) {
    cap.init(buf.data(), n, anchors.data(), an, boot);
  }
  void push(uint32_t n, int base = 0) {
    for (uint32_t i = 0; i < n; i++) {
      int16_t s[6];
      for (int j = 0; j < 6; j++) s[j] = int16_t((base + int(i)) * 7 + j);
      REQUIRE(cap.push(s));
    }
  }
  void freeze_all() {
    REQUIRE(cap.begin_freeze(PlatformImuFreezeReason::ATTEMPT_END));
    while (!cap.crc_step(8192)) {}
    REQUIRE(cap.finalize(info));
  }
};

// CRC-32 of the payload words read back through read_word in seq order, and the header words.
uint32_t walk_payload_crc(const Capture& c) {
  uint32_t crc = 0xFFFFFFFFu;
  uint8_t w[6];
  for (uint32_t p = 0; p < 3 * c.n_anchors(); p++) { REQUIRE(c.read_word(PlatformImu::ANCHOR_SEQ_BASE + p, w)); crc = crc32_update(crc, w, 6); }
  for (uint32_t p = 0; p < 2 * c.n_samples(); p++) { REQUIRE(c.read_word(p, w)); crc = crc32_update(crc, w, 6); }
  return ~crc;
}

}  // namespace

TEST_CASE("B1: the appended counter wrapping past 2^32 keeps WRAPPED and the anchor trimming right") {
  SmallRig r(100, 8);
  r.cap.test_seed_appended(0xFFFFFF8Du);  // 115 samples before the u32 wrap
  r.push(10);
  r.cap.add_anchor(1, 1, 0);              // index 0xFFFFFF97: older than the oldest retained sample at freeze
  r.push(20);
  r.cap.add_anchor(2, 2, 0);              // index 0xFFFFFFAB: retained
  r.push(90);                             // 120 pushed in all: the counter has wrapped to 5, the ring is full
  r.cap.add_anchor(3, 3, 0);              // index 5: retained
  CHECK(r.cap.appended() == 5u);          // appended_ <= capacity although 120 samples went through a 100 ring
  CHECK(r.cap.wrapped());                 // the old `appended_ > cap_` test said false here
  r.freeze_all();
  CHECK(r.cap.n_samples() == 100u);
  CHECK(r.cap.n_anchors() == 2u);         // the old unsigned compare kept the stale one and dropped the newest
  const uint8_t* h = r.cap.header();
  CHECK(get32(h + 12) == 0xFFFFFFA1u);    // first_index: appended 5 minus 100 retained, modulo 2^32
  CHECK(flag16(h + 18, PlatformImuCaptureFlag::WRAPPED));
  uint8_t rec[18];
  for (int k = 0; k < 3; k++) REQUIRE(r.cap.read_word(PlatformImu::ANCHOR_SEQ_BASE + k, rec + 6 * k));
  CHECK(get32(rec) == 0xFFFFFFABu);
  for (int k = 0; k < 3; k++) REQUIRE(r.cap.read_word(PlatformImu::ANCHOR_SEQ_BASE + 3 + k, rec + 6 * k));
  CHECK(get32(rec) == 5u);
  CHECK(get32(h + 54) == walk_payload_crc(r.cap));
}

TEST_CASE("exactly capacity is not wrapped; capacity + 1 is, with the oldest sample dropped") {
  Rig r;
  go_running(r);
  r.push(CAP);
  CHECK_FALSE(r.cap.wrapped());
  Reply f = r.freeze();
  CHECK(f.detail == CAP);
  uint8_t w[6];
  REQUIRE(r.cap.read_word(0, w));
  CHECK(int16_t(get16(w)) == 0);                       // sample 0 is the oldest
  CHECK(get32(r.cap.header() + 12) == 0u);
  CHECK_FALSE(flag16(r.cap.header() + 18, PlatformImuCaptureFlag::WRAPPED));
  CHECK(get32(r.cap.header() + 54) == walk_payload_crc(r.cap));
  Rig q;
  go_running(q);
  q.push(CAP + 1);
  CHECK(q.cap.wrapped());
  q.freeze();
  REQUIRE(q.cap.read_word(0, w));
  CHECK(int16_t(get16(w)) == int16_t(1 * 7));          // sample 0 was overwritten: sample 1 is the oldest
  REQUIRE(q.cap.read_word(2 * CAP - 1, w));
  CHECK(int16_t(get16(w + 4)) == int16_t(CAP * 7 + 5));
  CHECK(get32(q.cap.header() + 12) == 1u);
  CHECK(flag16(q.cap.header() + 18, PlatformImuCaptureFlag::WRAPPED));
  CHECK(get32(q.cap.header() + 54) == walk_payload_crc(q.cap));
}

TEST_CASE("one sample and no anchors freeze cleanly") {
  Rig r;
  go_running(r);
  r.push(1);
  Reply f = r.freeze();
  CHECK(f.status == St::OK);
  CHECK(f.detail == 1u);
  CHECK(r.cap.n_anchors() == 0u);
  uint8_t w[6];
  CHECK_FALSE(r.cap.read_word(2, w));                           // past the sample region
  CHECK_FALSE(r.cap.read_word(PlatformImu::ANCHOR_SEQ_BASE, w)); // no anchor words
  CHECK(r.cap.read_word(1, w));
  CHECK(get32(r.cap.header() + 54) == walk_payload_crc(r.cap));
  CHECK(get32(r.cap.header() + 62) == crc32(r.cap.header(), 62));
}

TEST_CASE("anchor ring wrapped (head != 0) and trimmed in the same freeze") {
  SmallRig r(50, 4);
  for (int i = 1; i <= 13; i++) { r.push(10, i * 10); r.cap.add_anchor(uint32_t(i), uint64_t(i), uint8_t(i)); }  // idx 10..130
  r.push(30, 200);                                    // appended 160: first_index 110
  r.freeze_all();
  // The ring holds the last four anchors (idx 100,110,120,130), written with a_head_ = 13 % 4 = 1.
  // Anchors at idx <= 110 refer to samples no longer retained: 100 and 110 are dropped.
  CHECK(r.cap.n_anchors() == 2u);
  const uint8_t* h = r.cap.header();
  CHECK(get16(h + 16) == 2u);
  CHECK(get32(h + 12) == 110u);
  uint8_t rec[18];
  for (int k = 0; k < 3; k++) REQUIRE(r.cap.read_word(PlatformImu::ANCHOR_SEQ_BASE + k, rec + 6 * k));
  CHECK(get32(rec) == 120u);
  CHECK(rec[16] == 12);
  for (int k = 0; k < 3; k++) REQUIRE(r.cap.read_word(PlatformImu::ANCHOR_SEQ_BASE + 3 + k, rec + 6 * k));
  CHECK(get32(rec) == 130u);
  CHECK(rec[16] == 13);
  CHECK(get32(h + 54) == walk_payload_crc(r.cap));
  // chunked CRC equals the one-shot walk (freeze_all used 8192-byte slices; redo with 6-byte slices)
  SmallRig q(50, 4);
  for (int i = 1; i <= 13; i++) { q.push(10, i * 10); q.cap.add_anchor(uint32_t(i), uint64_t(i), uint8_t(i)); }
  q.push(30, 200);
  REQUIRE(q.cap.begin_freeze(1));
  while (!q.cap.crc_step(6)) {}
  CHECK(q.cap.payload_crc() == r.cap.payload_crc());
}

namespace {
// The real sizes, in static storage (384 KB of samples, 400 anchors): the shipped DMAMEM layout.
int16_t g_real_buf[PlatformImu::RING_SAMPLES * 6];
Anchor g_real_anchors[PlatformImu::ANCHOR_RING];

void real_push(Capture& c, uint32_t n, uint32_t base, uint32_t anchor_every) {
  for (uint32_t i = 0; i < n; i++) {
    int16_t s[6];
    for (int j = 0; j < 6; j++) s[j] = int16_t((base + i) * 7 + j);
    REQUIRE(c.push(s));
    if (anchor_every && (c.appended() % anchor_every) == 0) c.add_anchor(c.appended(), c.appended(), 0);
  }
}
}  // namespace

TEST_CASE("real sizes: 32000 samples fill the 16-bit seq space exactly, with 320 anchors and the header") {
  Capture c;
  c.init(g_real_buf, PlatformImu::RING_SAMPLES, g_real_anchors, PlatformImu::ANCHOR_RING, 0x0BADF00Du);
  real_push(c, PlatformImu::RING_SAMPLES, 0, 100);
  CHECK_FALSE(c.wrapped());
  REQUIRE(c.begin_freeze(1));
  while (!c.crc_step(8192)) {}
  HeaderInfo hi;
  REQUIRE(c.finalize(hi));
  CHECK(c.n_samples() == 32000u);
  CHECK(c.n_anchors() == 320u);
  uint8_t w[6];
  // last sample word: seq 63999 is the gyro half of sample 31999; it round-trips through pack_data's u16 seq
  REQUIRE(c.read_word(63999, w));
  CHECK(int16_t(get16(w + 4)) == int16_t(31999u * 7u + 5u));
  Frame fr = pack_data(uint16_t(63999), w);
  CHECK(get16(fr.d) == 63999);
  CHECK_FALSE(c.read_word(64000, w));
  // last anchor word, and one past it
  uint32_t last_anchor_seq = PlatformImu::ANCHOR_SEQ_BASE + 3u * 320u - 1u;
  CHECK(last_anchor_seq == 65087u);
  CHECK(c.read_word(last_anchor_seq, w));
  CHECK_FALSE(c.read_word(last_anchor_seq + 1, w));
  // header words at 65520..65530
  for (uint32_t k = 0; k < 11; k++) CHECK(c.read_word(PlatformImu::HEADER_SEQ_BASE + k, w));
  CHECK_FALSE(c.read_word(PlatformImu::HEADER_SEQ_BASE + 11, w));
  CHECK(get32(c.header() + 54) == walk_payload_crc(c));
  CHECK(get32(c.header() + 62) == crc32(c.header(), 62));
  CHECK(get32(c.header() + 8) == 32000u);
  CHECK(get16(c.header() + 16) == 320u);
}

TEST_CASE("real sizes: 32001 samples wrap by one; 64000 samples with 640 anchors wrap the anchor ring and trim") {
  Capture c;
  c.init(g_real_buf, PlatformImu::RING_SAMPLES, g_real_anchors, PlatformImu::ANCHOR_RING, 0x0BADF00Du);
  real_push(c, PlatformImu::RING_SAMPLES + 1, 0, 100);
  CHECK(c.wrapped());
  REQUIRE(c.begin_freeze(1));
  while (!c.crc_step(8192)) {}
  HeaderInfo hi;
  REQUIRE(c.finalize(hi));
  CHECK(get32(c.header() + 12) == 1u);
  CHECK(c.n_anchors() == 320u);   // anchors at 100..32000, all of which reference retained samples
  uint8_t w[6];
  REQUIRE(c.read_word(0, w));
  CHECK(int16_t(get16(w)) == int16_t(1u * 7u));
  CHECK(get32(c.header() + 54) == walk_payload_crc(c));

  Capture d;
  d.init(g_real_buf, PlatformImu::RING_SAMPLES, g_real_anchors, PlatformImu::ANCHOR_RING, 0x0BADF00Du);
  real_push(d, 2 * PlatformImu::RING_SAMPLES, 0, 100);  // 640 anchors into a 400 ring: head = 240, then trimmed
  REQUIRE(d.begin_freeze(1));
  while (!d.crc_step(8192)) {}
  REQUIRE(d.finalize(hi));
  CHECK(d.wrapped());
  CHECK(get32(d.header() + 12) == 32000u);
  CHECK(d.n_anchors() == 320u);   // ring kept idx 24100..64000; those <= 32000 are dropped: 32100..64000 remain
  uint8_t rec[18];
  for (int k = 0; k < 3; k++) REQUIRE(d.read_word(PlatformImu::ANCHOR_SEQ_BASE + k, rec + 6 * k));
  CHECK(get32(rec) == 32100u);
  uint32_t last = PlatformImu::ANCHOR_SEQ_BASE + 3u * 320u - 3u;
  for (int k = 0; k < 3; k++) REQUIRE(d.read_word(last + k, rec + 6 * k));
  CHECK(get32(rec) == 64000u);
  CHECK(get32(d.header() + 54) == walk_payload_crc(d));
  CHECK(get32(d.header() + 62) == crc32(d.header(), 62));
}

TEST_CASE("B6: capacities beyond the address-space limits are clamped") {
  static int16_t dummy[6];
  static Anchor adummy[1];
  Capture c;
  c.init(dummy, 1000000u, adummy, 1000000u, 1u);
  CHECK(c.capacity() == PlatformImu::RING_SAMPLES);
  // 12 / 6 / 3 / 62 are derived from the generated constants
  CHECK(Capture::SAMPLE_WORDS == 2u);
  CHECK(Capture::ANCHOR_WORDS == 3u);
  CHECK(Capture::HALF_INT16 == 3u);
  CHECK(Capture::HEADER_CRC_SPAN == 62u);
}

TEST_CASE("B2: after RESUME sampling() is false until the FIFO flush has been serviced") {
  Rig r;
  go_running(r);
  CHECK(r.eng.sampling());
  r.push(10);
  uint16_t id = r.freeze().capture_id;
  CHECK_FALSE(r.eng.sampling());                       // FROZEN
  REQUIRE(r.resume(id).status == St::OK);
  CHECK(r.eng.state() == S::RUNNING);
  CHECK_FALSE(r.eng.sampling());                       // the chip's FIFO still holds the frozen interval
  CHECK(r.eng.take_resume_check());                    // the sketch flushes the FIFO now
  CHECK(r.eng.sampling());
  CHECK_FALSE(r.eng.take_resume_check());
}

TEST_CASE("B2: a resume flush or a discontinuity pending from a dead capture never leaks into the next one") {
  // DISABLE then ENABLE
  Rig a;
  go_running(a);
  a.push(5);
  uint16_t id = a.freeze().capture_id;
  a.resume(id);                                        // resume_check_ pending
  a.eng.note_overflow();                               // disc_pending_ too
  REQUIRE(a.send(Op::DISABLE).status == St::OK);
  REQUIRE(a.send(Op::ENABLE).status == St::OK);
  a.eng.on_chip_ready();
  CHECK(a.eng.sampling());                             // not held back by the old capture's flush
  CHECK_FALSE(a.eng.take_resume_check());
  CHECK_FALSE(a.eng.take_discontinuity());
  // firmware update begin / abandon
  Rig b;
  go_running(b);
  b.push(5);
  id = b.freeze().capture_id;
  b.resume(id);
  b.eng.note_overflow();
  b.eng.on_fw_update_begin();
  b.eng.on_fw_update_abandoned();
  b.eng.on_chip_ready();
  CHECK(b.eng.state() == S::RUNNING);
  CHECK(b.eng.sampling());
  CHECK_FALSE(b.eng.take_resume_check());
  CHECK_FALSE(b.eng.take_discontinuity());
  // a discontinuity noted before a freeze does not mark the capture that follows RESUME
  Rig c;
  go_running(c);
  c.push(5);
  c.eng.on_i2c_error(0);                               // BACKOFF + disc_pending_
  id = c.freeze().capture_id;
  REQUIRE(c.resume(id).status == St::OK);
  CHECK(c.eng.take_resume_check());
  CHECK_FALSE(c.eng.take_discontinuity());
  // init() on a reused engine
  Rig d;
  go_running(d);
  d.push(5);
  id = d.freeze().capture_id;
  d.resume(id);
  d.eng.init(&d.cap);
  CHECK_FALSE(d.eng.take_resume_check());
}

TEST_CASE("B3: every reply carries the frozen capture's id, refusals and ABORT included") {
  Rig r;
  go_running(r);
  r.push(10);
  CHECK(r.send(Op::ABORT).capture_id == 0);            // nothing frozen
  uint16_t id = r.freeze().capture_id;
  REQUIRE(id != 0);
  CHECK(r.send(Op::ENABLE).capture_id == id);          // ENABLE while frozen
  CHECK(r.send(Op::ABORT).capture_id == id);           // ABORT while merely frozen
  CHECK(r.send(Op::FREEZE, 1).capture_id == id);       // ALREADY_FROZEN
  CHECK(r.send(Op::READ, 0xFF, 0xFF, 0, 0, 1, 0, 100).capture_id == id);  // ERR_WRONG_CAPTURE still names the frozen one
  CHECK(r.send(Op::FREEZE, 0).capture_id == id);       // ERR_BAD_ARGS
  REQUIRE(r.read(id, 0, 6).status == St::OK);
  CHECK(r.send(Op::ENABLE).capture_id == id);          // ENABLE while UPLOADING
  Reply ab = r.send(Op::ABORT);                        // the cut upload's reply correlates with its capture
  CHECK(ab.capture_id == id);
  CHECK(r.eng.state() == S::FROZEN);
  Reply dis = r.send(Op::DISABLE);                     // the capture is discarded
  CHECK(dis.capture_id == 0);
  CHECK(r.send(Op::ENABLE).capture_id == 0);
}

TEST_CASE("B7: the gap floor holds between consecutive READ blocks and a replacing READ") {
  Rig r;
  go_running(r);
  r.push(10);
  uint16_t id = r.freeze().capture_id;
  uint32_t t = 0xFFFFFC00u;
  Frame f{};
  REQUIRE(r.read(id, 0, 2, 100).status == St::OK);     // gap 1000 us
  REQUIRE(r.eng.peek_data(t, &f));
  r.eng.commit_data(t);
  REQUIRE(r.eng.peek_data(t + 1000, &f));
  r.eng.commit_data(t + 1000);                         // block done at t + 1000
  CHECK(r.eng.state() == S::FROZEN);
  // a new block asked for 100 us later: its first frame waits out the gap since the previous block's last frame
  REQUIRE(r.read(id, 4, 2, 100).status == St::OK);
  CHECK_FALSE(r.eng.peek_data(t + 1100, &f));
  CHECK(r.eng.data_wait_us(t + 1100) == 900u);
  CHECK_FALSE(r.eng.peek_data(t + 1999, &f));
  REQUIRE(r.eng.peek_data(t + 2000, &f));              // across the u32 wrap
  CHECK(get16(f.d) == 4);
  r.eng.commit_data(t + 2000);
  // a replacing READ mid-block does not reset the floor either
  REQUIRE(r.read(id, 8, 1, 100).status == St::OK);
  CHECK_FALSE(r.eng.peek_data(t + 2500, &f));
  REQUIRE(r.eng.peek_data(t + 3000, &f));
  CHECK(get16(f.d) == 8);
  // long after the last frame (idle for a minute), the first frame of a block is due at once
  r.eng.commit_data(t + 3000);
  REQUIRE(r.read(id, 0, 1, 100).status == St::OK);
  REQUIRE(r.eng.peek_data(t + 60u * 1000000u, &f));
}

TEST_CASE("T3: refused READs leave a running block running; FREEZE in UPLOADING does not cut it") {
  Rig r;
  go_running(r);
  r.push(10);
  uint16_t id = r.freeze().capture_id;
  REQUIRE(r.read(id, 2, 6).status == St::OK);
  Frame f{};
  CHECK(r.send(Op::FREEZE, 1).status == St::ALREADY_FROZEN);
  CHECK(r.eng.block_active());
  CHECK(r.eng.state() == S::UPLOADING);
  CHECK(r.read(uint16_t(id + 1), 0, 1).status == St::ERR_WRONG_CAPTURE);
  CHECK(r.eng.block_active());
  Reply oor = r.read(id, 19, 5);                       // straddles the end of the sample region (20 words)
  CHECK(oor.status == St::ERR_OUT_OF_RANGE);
  CHECK((oor.detail & 0xFFFFu) == 19u);
  CHECK((oor.detail >> 16) == 20u);                    // region_end
  CHECK(r.eng.block_active());
  CHECK(r.eng.state() == S::UPLOADING);
  REQUIRE(r.eng.peek_data(0, &f));
  CHECK(get16(f.d) == 2);                              // still the original block
  Reply none = r.read(id, 1000, 1);                    // a seq in no region
  CHECK(none.status == St::ERR_OUT_OF_RANGE);
  CHECK((none.detail >> 16) == 0u);                    // region_end = 0
}

TEST_CASE("T3: DISABLE from FROZEN discards; RESUME of a capture frozen from BACKOFF runs again") {
  Rig r;
  go_running(r);
  r.push(10);
  r.freeze();
  REQUIRE(r.send(Op::DISABLE).status == St::OK);
  CHECK(r.eng.state() == S::DISABLED);
  CHECK(r.cap.n_samples() == 0u);
  CHECK_FALSE(r.cap.frozen());
  Rig q;
  go_running(q);
  q.push(10);
  q.eng.on_i2c_error(0);
  REQUIRE(q.eng.state() == S::BACKOFF);
  Reply f = q.freeze();
  REQUIRE(f.status == St::OK);
  CHECK(flag16(q.cap.header() + 18, PlatformImuCaptureFlag::SENSOR_ERROR));
  REQUIRE(q.resume(f.capture_id).status == St::OK);
  CHECK(q.eng.state() == S::RUNNING);
  CHECK(q.eng.take_resume_check());
}

TEST_CASE("T3: while the freeze CRC is pending a sample READ is accepted and the header is not readable") {
  Rig r;
  go_running(r);
  r.push(CAP);
  bool def = false;
  r.send(Op::FREEZE, 1, 0, 0, 0, 0, 0, 0, &def);
  REQUIRE(def);
  REQUIRE(r.eng.freeze_pending());
  uint16_t id = r.cap.capture_id();
  Reply hdr = r.read(id, uint16_t(PlatformImu::HEADER_SEQ_BASE), 1);
  CHECK(hdr.status == St::ERR_OUT_OF_RANGE);
  CHECK((hdr.detail >> 16) == 0u);
  CHECK(r.read(id, 0, 4).status == St::OK);
  Reply out;
  while (!r.eng.service_freeze(&out)) {}
  CHECK(r.read(id, uint16_t(PlatformImu::HEADER_SEQ_BASE), 1).status == St::OK);  // readable once finalized
}

/* ------------------------------- FW 13 ------------------------------- */

namespace {
// Drain a block with passes every `step_us`, committing each due frame; returns the send times.
std::vector<uint32_t> drain_block(Rig& r, uint32_t& t, uint32_t step_us, uint32_t max_us,
                                  std::vector<uint16_t>* seqs = nullptr) {
  std::vector<uint32_t> times;
  const uint32_t end = t + max_us;
  Frame f{};
  while (r.eng.block_active() && int32_t(t - end) < 0) {
    if (r.eng.peek_data(t, &f)) {
      r.eng.commit_data(t);
      times.push_back(t);
      if (seqs) seqs->push_back(get16(f.d));
    }
    t += step_us;
  }
  return times;
}
Reply set_odr(Rig& r, uint8_t khz) { return r.send(Op::SET_ODR, khz); }
}  // namespace

TEST_CASE("FW 13 parse_cmd SET_ODR: 1 and 2 kHz pass; 0, 3, 255 and a short frame fail") {
  const uint8_t ok1[2] = {Op::SET_ODR, 1}, ok2[2] = {Op::SET_ODR, 2};
  CHECK(parse_cmd(ok1, 2).valid); CHECK(parse_cmd(ok1, 2).odr_hz == 1000);
  CHECK(parse_cmd(ok2, 2).valid); CHECK(parse_cmd(ok2, 2).odr_hz == 2000);
  for (uint8_t k : {uint8_t(0), uint8_t(3), uint8_t(255)}) {
    const uint8_t d[2] = {Op::SET_ODR, k};
    CHECK_FALSE(parse_cmd(d, 2).valid);
  }
  CHECK_FALSE(parse_cmd(ok2, 1).valid);
  CHECK(PlatformImu::FAST_ODR_HZ == 2000);
}

TEST_CASE("FW 13 SET_ODR legality: idempotent, refused while frozen, restarts the capture otherwise") {
  {  // RUNNING: a new rate discards the capture and re-probes; the same rate again changes nothing
    Rig r;
    go_running(r);
    r.push(10);
    CHECK(r.eng.odr_hz() == 1000);
    Reply a = set_odr(r, 1);
    CHECK(a.status == St::OK); CHECK(a.detail == 1000);
    CHECK(r.eng.state() == S::RUNNING); CHECK(r.cap.n_samples() == 10); CHECK_FALSE(r.eng.take_rate_change());
    Reply b = set_odr(r, 2);
    CHECK(b.status == St::OK); CHECK(b.detail == 2000); CHECK(b.capture_id == 0);
    CHECK(r.eng.odr_hz() == 2000);
    CHECK(r.eng.state() == S::PROBING);
    CHECK(r.cap.n_samples() == 0);
    CHECK(r.eng.take_rate_change()); CHECK_FALSE(r.eng.take_rate_change());
    CHECK_FALSE(r.eng.take_discontinuity());          // a fresh capture, not a gap in the old one
    CHECK(set_odr(r, 2).status == St::OK);            // the retry: no second restart
    CHECK_FALSE(r.eng.take_rate_change());
  }
  {  // FROZEN and UPLOADING: ERR_FROZEN, nothing changes (the capture is not yet uploaded)
    Rig r;
    go_running(r);
    r.push(10);
    const uint16_t id = r.freeze().capture_id;
    Reply f = set_odr(r, 2);
    CHECK(f.status == St::ERR_FROZEN); CHECK(f.capture_id == id);
    CHECK(r.eng.odr_hz() == 1000); CHECK(r.eng.state() == S::FROZEN); CHECK(r.cap.frozen());
    REQUIRE(r.read(id, 0, 4).status == St::OK);
    CHECK(set_odr(r, 2).status == St::ERR_FROZEN);
    CHECK(r.eng.state() == S::UPLOADING); CHECK(r.eng.block_active());
    CHECK(set_odr(r, 1).status == St::OK);            // the rate it already has: fine even when frozen
  }
  {  // DISABLED stores the rate for the next ENABLE; BACKOFF and PROBING keep their state
    Rig r;
    r.send(Op::DISABLE);
    CHECK(set_odr(r, 2).status == St::OK);
    CHECK(r.eng.state() == S::DISABLED); CHECK(r.eng.odr_hz() == 2000); CHECK_FALSE(r.eng.take_rate_change());
    CHECK(r.send(Op::ENABLE).status == St::OK);
    CHECK(r.eng.odr_hz() == 2000);
    go_running(r);
    r.push(3);
    r.eng.on_i2c_error(0);
    REQUIRE(r.eng.state() == S::BACKOFF);
    CHECK(set_odr(r, 1).status == St::OK);
    CHECK(r.eng.state() == S::BACKOFF); CHECK(r.cap.n_samples() == 0); CHECK(r.eng.take_rate_change());
  }
  {  // FW_HALTED: ERR_FW_UPDATE; a 400 kHz build caps the rate at 1 kHz
    Rig r;
    r.eng.on_fw_update_begin();
    CHECK(set_odr(r, 2).status == St::ERR_FW_UPDATE);
    CHECK(r.eng.odr_hz() == 1000);
    Rig q;
    q.eng.set_max_odr_hz(1000);
    CHECK(set_odr(q, 2).status == St::ERR_BAD_ARGS);
    CHECK(set_odr(q, 1).status == St::OK);
  }
}

TEST_CASE("FW 13 READ chaining: a contiguous READ extends the block; anything else replaces it") {
  Rig r;
  go_running(r);
  r.push(50);                                          // 100 sample words
  const uint16_t id = r.freeze().capture_id;
  uint32_t t = 0xFFFFF000u;                            // across the u32 wrap
  REQUIRE(r.read(id, 0, 20, 80).status == St::OK);
  std::vector<uint16_t> seqs;
  Frame f{};
  REQUIRE(r.eng.peek_data(t, &f)); r.eng.commit_data(t); seqs.push_back(get16(f.d));
  Reply e = r.read(id, 20, 20, 80);                    // keeps one READ ahead
  CHECK(e.status == St::READ_EXTENDED);
  CHECK(e.capture_id == id);
  CHECK(e.detail == (20u | (20u << 16)));
  Reply e2 = r.read(id, 40, 10, 80);                   // and another
  CHECK(e2.status == St::READ_EXTENDED);
  t += 800;
  auto times = drain_block(r, t, 50, 1000000, &seqs);
  REQUIRE(seqs.size() == 50);
  for (size_t i = 0; i < seqs.size(); i++) CHECK(seqs[i] == i);       // one stream, in order
  for (size_t i = 1; i < times.size(); i++) CHECK(times[i] - times[i - 1] == 800u);  // no restart at the joins
  CHECK(r.eng.state() == S::FROZEN);
  // After the block ended, the same READ is a fresh block (OK, not EXTENDED).
  REQUIRE(r.read(id, 50, 5, 80).status == St::OK);
  // A retried READ (first_seq inside the active block) replaces it from its own first_seq.
  REQUIRE(r.eng.peek_data(t + 800, &f)); r.eng.commit_data(t + 800);
  CHECK(r.read(id, 50, 5, 80).status == St::OK);
  CHECK(r.eng.block_active());
  seqs.clear();
  t += 1600;
  drain_block(r, t, 50, 1000000, &seqs);
  REQUIRE(seqs.size() == 5);
  CHECK(seqs[0] == 50);
  // A READ that would be contiguous but runs past the region end is ERR_OUT_OF_RANGE and leaves the block.
  REQUIRE(r.read(id, 90, 5, 80).status == St::OK);
  CHECK(r.read(id, 95, 6, 80).status == St::ERR_OUT_OF_RANGE);
  CHECK(r.read(id, 95, 5, 80).status == St::READ_EXTENDED);
  // ABORT ends a chained block like any other.
  CHECK(r.send(Op::ABORT).status == St::OK);
  CHECK_FALSE(r.eng.block_active());
}

TEST_CASE("FW 13 pacing: the delivered pace equals the requested gap under loop() jitter") {
  Rig r;
  go_running(r);
  r.push(100);
  const uint16_t id = r.freeze().capture_id;
  uint32_t t = 1000;
  REQUIRE(r.read(id, 0, 200, 80).status == St::OK);    // 800 us
  // Passes every 37 us: each frame goes out up to 36 us late. FW 12 measured from the send, so the
  // pace stretched to about 800 + the mean lateness; the schedule keeps it at exactly 800.
  auto times = drain_block(r, t, 37, 10000000);
  REQUIRE(times.size() == 200);
  const double pace = double(times.back() - times.front()) / double(times.size() - 1);
  CHECK(pace == doctest::Approx(800.0).epsilon(0.001));
  for (size_t i = 1; i < times.size(); i++) {
    CHECK(times[i] - times[i - 1] >= 800u - 37u);      // never faster than the gap less one pass
    CHECK(times[i] - times[i - 1] <= 800u + 37u);
  }
}

TEST_CASE("FW 13 pacing: after a stall at most MAX_CATCHUP_FRAMES go faster than the gap, in any window") {
  CHECK(Engine::MAX_CATCHUP_FRAMES == 2u);
  for (uint32_t stall : {500u, 1700u, 50000u, 3000000u}) {
    CAPTURE(stall);
    Rig r;
    go_running(r);
    r.push(100);
    const uint16_t id = r.freeze().capture_id;
    uint32_t t = 0xFFFF0000u;
    REQUIRE(r.read(id, 0, 200, 100).status == St::OK);  // 1000 us
    std::vector<uint32_t> times;
    Frame f{};
    int sent = 0;
    while (r.eng.block_active()) {
      if (sent == 20) { t += stall; sent++; }           // one long pass, mid-block
      if (r.eng.peek_data(t, &f)) { r.eng.commit_data(t); times.push_back(t); if (sent < 20) sent++; }
      t += 10;
    }
    REQUIRE(times.size() == 200);
    // The burst: frames that follow the previous one by much less than the gap (here one 10 us pass).
    int burst = 0;
    for (size_t i = 1; i < times.size(); i++) if (times[i] - times[i - 1] < 500u) burst++;
    CHECK(burst == int(stall >= 2000u ? Engine::MAX_CATCHUP_FRAMES : (stall >= 1000u ? 1u : 0u)));
    // The window bound: frames in any window W <= W / gap + 1 + MAX_CATCHUP_FRAMES.
    for (size_t i = 0; i < times.size(); i++)
      for (size_t j = i; j < times.size(); j++) {
        const uint32_t w = times[j] - times[i];
        CHECK(uint32_t(j - i + 1) <= w / 1000u + 1u + Engine::MAX_CATCHUP_FRAMES);
      }
  }
}

TEST_CASE("FW 13 pacing: a fresh block after idle starts its schedule at its first frame, no burst") {
  Rig r;
  go_running(r);
  r.push(100);
  const uint16_t id = r.freeze().capture_id;
  uint32_t t = 5000;
  REQUIRE(r.read(id, 0, 5, 80).status == St::OK);
  drain_block(r, t, 10, 1000000);
  t += 200000;                                          // the Jetson's turnaround, 200 ms idle
  REQUIRE(r.read(id, 10, 10, 80).status == St::OK);
  auto times = drain_block(r, t, 10, 1000000);
  REQUIRE(times.size() == 10);
  for (size_t i = 1; i < times.size(); i++) CHECK(times[i] - times[i - 1] == 800u);
  CHECK(r.eng.data_wait_us(times.back()) == 0u);       // block done
}
