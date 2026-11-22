// =============================================================================
//  test_platform_imu_fw14.cpp — Platform FW 14: the chunk-CRC table and the TX diag counters
// =============================================================================
//  Runs the shipped imu_capture.h / imu_service.h (no Arduino). Contract:
//  ros_ws/docs/platform_imu_protocol.md sections 3 and 6 (FW 14). Logbook:
//  2026-11-23-platform-imu-fw14-span-checksum-and-tx-counters.
//
//  Chunk table: every entry equals CRC-32/IEEE (zlib) of exactly the words of its chunk, read back
//  through the same READ path (read_word), for full and partial rings at the 1 kHz and 2 kHz anchor
//  counts, a wrapped ring, chunks that end early (one word, one sample), and empty chunks; the table
//  is computed lazily (nothing before the header, entries not READY until stepped) and only in loop()
//  passes that did nothing else. Diag: the mailbox tracker's verdicts against a faked mailbox state,
//  the ECR deltas, the 30-byte layout, and the snapshot a diag READ returns through the Service.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <array>
#include <cstring>
#include <map>
#include <vector>

#include "../../../ros_ws/src/jugglebot/Teensy_code_platform/imu_service.h"

using namespace ImuService;
using ImuCapture::Capture;
namespace Op = PlatformImuOp;
namespace St = PlatformImuStatus;
namespace S = PlatformImuState;

namespace {

uint16_t g16(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }
uint32_t g32(const uint8_t* p) { return uint32_t(g16(p)) | (uint32_t(g16(p + 2)) << 16); }

constexpr uint32_t CB = PlatformImu::CHUNK_CRC_SEQ_BASE;
constexpr uint32_t CW = PlatformImu::CHUNK_CRC_WORDS;
constexpr uint32_t READY = PlatformImu::CHUNK_CRC_READY;

struct Img {
  std::vector<int16_t> buf;
  std::vector<ImuCapture::Anchor> anchors;
  Capture cap;
  // `pushed` samples into a ring of `ring` (wraps when pushed > ring), then `n_anchors` anchors.
  Img(uint32_t ring, uint32_t pushed, uint32_t n_anchors) : buf(size_t(ring) * 6), anchors(PlatformImu::ANCHOR_RING) {
    cap.init(buf.data(), ring, anchors.data(), uint32_t(anchors.size()), 0xCAFE0014u);
    for (uint32_t k = 0; k < pushed; k++) {
      int16_t s[6];
      for (int j = 0; j < 6; j++) s[j] = int16_t((k * 131u + uint32_t(j) * 7919u) & 0xFFFF);
      REQUIRE(cap.push(s));
    }
    for (uint32_t a = 0; a < n_anchors; a++) REQUIRE(cap.add_anchor(1000u + a * 97u, 5000000000ull + a, uint8_t(a & 1)));
    REQUIRE(cap.begin_freeze(1));
    while (!cap.crc_step(8192)) {}
    ImuCapture::HeaderInfo h;
    h.fw_version = 14;
    REQUIRE(cap.finalize(h));
  }
  // zlib.crc32 of the region words [first, first + n), read back the way IMU_DATA carries them.
  uint32_t ref_crc(uint32_t first, uint32_t n) const {
    uint32_t c = 0xFFFFFFFFu;
    uint8_t w[6];
    for (uint32_t s = first; s < first + n; s++) {
      REQUIRE(cap.read_word(s, w));
      c = ImuCapture::crc32_update(c, w, 6);
    }
    return ~c;
  }
  void entry(uint32_t i, uint32_t* crc, uint32_t* info) const {
    uint8_t w[6];
    REQUIRE(cap.read_word(CB + i, w));
    *crc = g32(w);
    *info = g16(w + 4);
  }
  void finish_table() { int guard = 0; while (!cap.chunk_step(CHUNK_STEP_WORDS)) REQUIRE(++guard < 100000); }
  // Every entry is READY with the right length and the CRC of exactly its words.
  void check_table() const {
    const uint32_t sw = 2 * cap.n_samples(), aw = 3 * cap.n_anchors();
    for (uint32_t i = 0; i < ImuCapture::CHUNK_TABLE_WORDS; i++) {
      const bool smp = i < ImuCapture::SAMPLE_CHUNKS;
      const uint32_t total = smp ? sw : aw;
      const uint32_t start = (smp ? i : i - ImuCapture::SAMPLE_CHUNKS) * CW;
      const uint32_t n = start >= total ? 0 : (total - start < CW ? total - start : CW);
      uint32_t crc, info;
      entry(i, &crc, &info);
      CHECK(info == (n | READY));
      const uint32_t base = smp ? 0u : PlatformImu::ANCHOR_SEQ_BASE;
      CHECK(crc == (n ? ref_crc(base + start, n) : 0u));   // an empty chunk is crc32(b"") = 0
    }
  }
};

}  // namespace

TEST_CASE("the chunk table sits where the YAML says and is one READ") {
  CHECK(ImuCapture::SAMPLE_CHUNKS == 125);
  CHECK(ImuCapture::ANCHOR_CHUNKS == 3);
  CHECK(ImuCapture::CHUNK_TABLE_WORDS == 128);
  CHECK(CB == 0xFF40);
  CHECK(PlatformImu::DIAG_SEQ_BASE == 0xFA40);
}

TEST_CASE("full wrapped ring at 1 kHz (320 anchors): every entry matches its words") {
  Img im(PlatformImu::RING_SAMPLES, PlatformImu::RING_SAMPLES + 777, 320);
  CHECK(im.cap.wrapped());
  CHECK(im.cap.n_samples() == PlatformImu::RING_SAMPLES);
  im.finish_table();
  im.check_table();
  uint32_t crc, info;
  im.entry(ImuCapture::SAMPLE_CHUNKS + 1, &crc, &info);
  CHECK(info == ((960u - 512u) | READY));                  // the anchors' second chunk ends early
}

TEST_CASE("full wrapped ring at 2 kHz (160 anchors) and a partial ring with empty chunks") {
  Img a(PlatformImu::RING_SAMPLES, 2 * PlatformImu::RING_SAMPLES + 5, 160);
  a.finish_table();
  a.check_table();
  Img b(PlatformImu::RING_SAMPLES, 1000, 12);              // 2000 words: 3 full chunks + 464, then empty ones
  b.finish_table();
  b.check_table();
  uint32_t crc, info;
  b.entry(3, &crc, &info);
  CHECK(info == (464u | READY));
  b.entry(4, &crc, &info);
  CHECK(info == READY);
  CHECK(crc == 0u);
}

TEST_CASE("chunks that end after one word or one sample") {
  Img a(4096, 5000, 171);                                  // 513 anchor words: the second anchor chunk is 1 word
  a.finish_table();
  a.check_table();
  uint32_t crc, info;
  a.entry(ImuCapture::SAMPLE_CHUNKS + 1, &crc, &info);
  CHECK(info == (1u | READY));
  CHECK(crc == a.ref_crc(PlatformImu::ANCHOR_SEQ_BASE + 512, 1));
  Img b(4096, 1, 1);                                       // one sample: a 2-word chunk
  b.finish_table();
  b.check_table();
  b.entry(0, &crc, &info);
  CHECK(info == (2u | READY));
}

TEST_CASE("lazy: nothing before the header, entries not READY until stepped, bounded slices") {
  std::vector<int16_t> buf(4096 * 6);
  std::vector<ImuCapture::Anchor> anc(64);
  Capture cap;
  cap.init(buf.data(), 4096, anc.data(), 64, 0x1234u);
  int16_t s[6] = {1, 2, 3, 4, 5, 6};
  for (int k = 0; k < 4096 + 10; k++) REQUIRE(cap.push(s));
  REQUIRE(cap.add_anchor(1, 2, 0));
  REQUIRE(cap.begin_freeze(1));
  CHECK_FALSE(cap.chunk_step(1u << 20));                   // payload CRC not done, no header: no work
  CHECK(cap.region_end(CB) == 0);                          // a READ of the table is ERR_OUT_OF_RANGE now
  CHECK(cap.region_end(PlatformImu::DIAG_SEQ_BASE) == 0);
  while (!cap.crc_step(8192)) {}
  ImuCapture::HeaderInfo h;
  REQUIRE(cap.finalize(h));
  CHECK(cap.range_valid(CB, ImuCapture::CHUNK_TABLE_WORDS));
  CHECK_FALSE(cap.range_valid(CB, ImuCapture::CHUNK_TABLE_WORDS + 1));
  CHECK(cap.range_valid(PlatformImu::DIAG_SEQ_BASE, PlatformImu::DIAG_WORDS));
  CHECK_FALSE(cap.range_valid(PlatformImu::DIAG_SEQ_BASE, PlatformImu::DIAG_WORDS + 1));
  uint8_t w[6];
  REQUIRE(cap.read_word(CB, w));
  CHECK(g16(w + 4) == CW);                                 // length known, READY clear, CRC 0
  CHECK(g32(w) == 0u);
  CHECK_FALSE(cap.chunk_step(CHUNK_STEP_WORDS));           // 8192 sample words: one slice is not enough
  CHECK(cap.chunks_ready() == 0);                          // 128 words < 512: still inside chunk 0
  for (int i = 0; i < 3; i++) cap.chunk_step(CHUNK_STEP_WORDS);
  CHECK(cap.chunks_ready() == 1);                          // exactly 4 slices per full chunk
  REQUIRE(cap.read_word(CB, w));
  CHECK(g16(w + 4) == (CW | READY));
  REQUIRE(cap.read_word(CB + 1, w));
  CHECK((g16(w + 4) & READY) == 0);
  // RESUME (reset) forgets the table; a new freeze starts it again.
  cap.reset();
  CHECK_FALSE(cap.chunks_done());
  CHECK(cap.chunks_ready() == 0);
}

/* -------------------------------- TX tracker -------------------------------- */

namespace {
struct FakeMb {
  std::map<int, MbState> mb;
  void operator()(int i, MbState* st) const { auto it = mb.find(i); *st = it == mb.end() ? MbState{} : it->second; }
};
MbState once(uint16_t seq, uint32_t id = PlatformCanId::IMU_DATA) { MbState s; s.code = MB_CODE_TX_ONCE; s.id = id; s.seq = seq; return s; }
MbState inactive() { MbState s; s.code = MB_CODE_TX_INACTIVE; return s; }
}  // namespace

TEST_CASE("tracker: pending, over 2 ms once, done with latency, abort, reuse by another frame") {
  TxTracker t;
  FakeMb f;
  t.on_placed(8, 100, 1000);
  t.on_placed(9, 101, 1100);
  t.on_placed(10, 102, 1200);
  f.mb[8] = once(100);
  f.mb[9] = once(101);
  f.mb[10] = once(102);
  t.poll(1500, f);
  CHECK(t.pending() == 3);
  CHECK(t.tx_done() == 0);
  t.poll(3050, f);                                         // MB 8 is 2.05 ms old: over the limit, counted once
  t.poll(3060, f);
  uint8_t d[30];
  t.pack(d, 3, 7);
  CHECK(g16(d + 12) == 1);
  f.mb[8] = inactive();                                    // sent
  f.mb[9] = once(555, 0x123);                              // sent, and the mailbox reused by another writer
  f.mb[10].code = 0x9;                                     // ABORT: lost in the mailbox
  t.poll(3400, f);
  CHECK(t.pending() == 0);
  CHECK(t.tx_done() == 2);
  t.pack(d, 3, 7);
  CHECK(d[0] == PlatformImu::DIAG_VERSION);
  CHECK(d[1] == 0x01);                                     // tracking valid
  CHECK(g32(d + 2) == 3);                                  // data_placed (the Service's count)
  CHECK(g32(d + 6) == 2);                                  // data_tx_done
  CHECK(g16(d + 10) == 1);                                 // data_tx_lost_mb
  CHECK(g16(d + 12) == 1);                                 // data_pend_over_2ms
  CHECK(g16(d + 14) == 3400 - 1000);                       // data_pend_max_us: MB 8 seen done at 3400
  CHECK(g16(d + 22) == 7);                                 // tx_deferred low 16 bits
  CHECK(g16(d + 24) == 0);                                 // tx_queued_anyway
  CHECK(g32(d + 26) == 0);                                 // reserved
  t.reset_pend_max();
  t.pack(d, 3, 7);
  CHECK(g16(d + 14) == 0);
}

TEST_CASE("tracker: a mailbox still holding the same frame stays pending; a predict miss clears valid") {
  TxTracker t;
  FakeMb f;
  t.on_placed(12, 7, 0);
  f.mb[12] = once(7);
  for (uint32_t us = 0; us < 10000; us += 500) t.poll(us, f);
  CHECK(t.pending() == 1);
  uint8_t d[30];
  t.pack(d, 1, 0);
  CHECK(g16(d + 12) == 1);                                 // once per frame, not once per poll
  CHECK(t.valid());
  t.note_predict_miss();
  CHECK_FALSE(t.valid());
  TxTracker u;
  u.on_placed(16, 1, 0);                                   // beyond setMaxMB(16): untrackable
  CHECK_FALSE(u.valid());
  u.note_queued_anyway();
  u.pack(d, 0, 0x12345);
  CHECK(d[1] == 0);
  CHECK(g16(d + 22) == 0x2345);
  CHECK(g16(d + 24) == 1);
}

TEST_CASE("tracker: ECR deltas are positive rises only, live values as read") {
  TxTracker t;
  t.note_ecr(0x0000);                                      // the baseline
  t.note_ecr(0x0308);                                      // TEC 8, REC 3
  t.note_ecr(0x0102);                                      // both fall: no increment
  t.note_ecr(0x0105);                                      // TEC +3
  uint8_t d[30];
  t.pack(d, 0, 0);
  CHECK(g16(d + 16) == 11);
  CHECK(g16(d + 18) == 3);
  CHECK(d[20] == 5);
  CHECK(d[21] == 1);
}

/* --------------------------- through the Service --------------------------- */

namespace {
struct Io {
  uint32_t t_us = 0, start = 0, consumed = 0;
  bool configured = false;
  int init_left = 0;
  uint16_t odr = 1000;
  uint32_t produced() const { return configured ? (t_us - start) / 1000u : 0; }
  void init_start(uint16_t) { init_left = 3; }
  int init_step(uint32_t now) { if (--init_left > 0) return INIT_PENDING; configured = true; start = now; consumed = 0; return 0; }
  int fifo_count(bool* ovf) { *ovf = false; uint32_t p = produced() - consumed; return int(p > 120 ? 120 : p); }
  int read_burst(Pkt* out, int n) {
    for (int i = 0; i < n; i++) { const uint32_t k = consumed++; for (int j = 0; j < 6; j++) out[i].s[j] = int16_t(k + j); out[i].temp = 9; }
    return n;
  }
  int flush() { consumed = produced(); return 0; }
  int check_who() { return 0; }
  void bus_recover() {}
  void stamp(Stamp* s) { s->local_us = t_us; s->wall_us = uint64_t(t_us) + 7000000000ull; s->wall_valid = true; }
  void header_info(ImuCapture::HeaderInfo* h) { h->odr_hz = 1000; h->fw_version = 14; }
};
struct Tx {
  std::vector<std::pair<uint32_t, std::array<uint8_t, 8>>> sent;
  bool place(uint32_t id, const uint8_t* d) { std::array<uint8_t, 8> a; std::memcpy(a.data(), d, 8); sent.push_back({id, a}); return true; }
  std::vector<std::array<uint8_t, 8>> of(uint32_t id) const { std::vector<std::array<uint8_t, 8>> v; for (auto& s : sent) if (s.first == id) v.push_back(s.second); return v; }
};
constexpr uint32_t CAP = 4000;
struct Rig {
  std::vector<int16_t> buf = std::vector<int16_t>(CAP * 6);
  std::vector<ImuCapture::Anchor> anc = std::vector<ImuCapture::Anchor>(64);
  Capture cap;
  Service svc;
  Io io;
  Tx tx;
  uint32_t t_us = 0;
  Rig() { cap.init(buf.data(), CAP, anc.data(), 64, 0xABCD0001u); svc.init(&cap, 0xABCD0001u, 0); }
  void pass() { io.t_us = t_us; svc.pass(t_us, t_us / 1000, false, io, tx); }
  void run_ms(uint32_t ms) { const uint32_t end = t_us + ms * 1000; while (t_us < end) { pass(); t_us += 100; } }
  void cmd(std::initializer_list<uint8_t> b) { uint8_t d[8] = {}; uint8_t n = 0; for (uint8_t x : b) d[n++] = x; svc.cmds().push(d, 8); }
  void read(uint16_t cid, uint16_t first, uint16_t count, uint8_t gap) {
    cmd({Op::READ, uint8_t(cid), uint8_t(cid >> 8), uint8_t(first), uint8_t(first >> 8), uint8_t(count), uint8_t(count >> 8), gap});
  }
};
}  // namespace

TEST_CASE("service: the chunk slice runs only in idle passes, the table completes while FROZEN, data pace holds") {
  Rig r;
  r.run_ms(BOOT_SETTLE_MS + 3000);
  REQUIRE(r.svc.state() == S::RUNNING);
  r.cmd({Op::FREEZE, 1});
  for (int i = 0; i < 2000 && !r.cap.header_ready(); i++) { r.pass(); r.t_us += 100; }
  REQUIRE(r.cap.header_ready());
  const uint16_t cid = r.cap.capture_id();
  // A READ in flight: no pass ever carries both a DATA frame and a chunk slice.
  r.read(cid, 0, 200, 40);
  int both = 0, chunk_passes = 0;
  size_t frames = 0;
  for (int i = 0; i < 2000; i++) {
    const size_t before = r.tx.of(PlatformCanId::IMU_DATA).size();
    const uint32_t ready_before = r.cap.chunks_ready();
    r.pass();
    const bool data = r.tx.of(PlatformCanId::IMU_DATA).size() > before;
    const bool chunk = r.svc.last_step() == Service::STEP_CHUNK;
    if (chunk) chunk_passes++;
    if (data && (chunk || r.cap.chunks_ready() != ready_before)) both++;
    r.t_us += 100;
  }
  frames = r.tx.of(PlatformCanId::IMU_DATA).size();
  CHECK(both == 0);
  CHECK(frames == 200);                                    // 200 ms at 0.4 ms per frame: the block is complete
  CHECK(chunk_passes > 0);
  CHECK(r.cap.chunks_done());
  CHECK(r.svc.data_placed() == 200);
}

TEST_CASE("service: a diag READ returns one snapshot of the counters; FW 13 regions unchanged") {
  Rig r;
  r.run_ms(BOOT_SETTLE_MS + 2000);
  r.cmd({Op::FREEZE, 1});
  for (int i = 0; i < 2000 && !r.cap.header_ready(); i++) { r.pass(); r.t_us += 100; }
  const uint16_t cid = r.cap.capture_id();
  r.read(cid, 0, 30, 40);
  r.run_ms(50);
  REQUIRE(r.tx.of(PlatformCanId::IMU_DATA).size() == 30);
  r.read(cid, uint16_t(PlatformImu::DIAG_SEQ_BASE), uint16_t(PlatformImu::DIAG_WORDS), 40);
  r.run_ms(20);
  auto rep = r.tx.of(PlatformCanId::IMU_REPLY).back();
  CHECK(rep[0] == Op::READ);
  CHECK(rep[1] == St::OK);
  auto data = r.tx.of(PlatformCanId::IMU_DATA);
  REQUIRE(data.size() == 35);
  uint8_t img[30];
  for (int i = 0; i < 5; i++) {
    const auto& f = data[30 + i];
    CHECK(g16(f.data()) == PlatformImu::DIAG_SEQ_BASE + uint32_t(i));
    std::memcpy(img + 6 * i, f.data() + 2, 6);
  }
  CHECK(img[0] == 1);
  CHECK(g32(img + 2) == 30);                               // the 30 frames placed BEFORE the diag READ
  CHECK(r.svc.data_placed() == 35);                        // the diag frames themselves count afterwards
  // A READ that overlaps the diag region's end is out of range, as for every region.
  r.read(cid, uint16_t(PlatformImu::DIAG_SEQ_BASE), uint16_t(PlatformImu::DIAG_WORDS + 1), 40);
  r.run_ms(5);
  rep = r.tx.of(PlatformCanId::IMU_REPLY).back();
  CHECK(rep[1] == St::ERR_OUT_OF_RANGE);
}

TEST_CASE("tracker: a mailbox reused before the poll credits the earlier frame as done, valid stays") {
  TxTracker t;
  t.on_placed(8, 100, 1000);
  t.on_placed(8, 101, 1800);                               // MB 8 reused before any poll saw frame 100 finish
  CHECK(t.valid());
  CHECK(t.tx_done() == 1);
  CHECK(t.pending() == 1);
  uint8_t d[30];
  t.pack(d, 2, 0);
  CHECK(d[1] == 0x01);
  CHECK(g16(d + 14) == 800);                               // latency of the old record, at the reuse time
}

TEST_CASE("tracker: on_written finds the real mailbox, prefers the prediction, counts a gone frame done") {
  TxTracker t;
  FakeMb f;
  f.mb[9] = once(7);                                       // the prediction (8) raced: frame 7 is in MB 9
  t.on_written(8, 8, 16, 7, 100, f);
  CHECK(t.valid());
  CHECK(t.pending() == 1);
  CHECK(t.tx_done() == 0);
  f.mb[8] = once(7);                                       // both hold seq 7: the predicted one wins
  f.mb[9] = inactive();
  TxTracker u;
  u.on_written(8, 8, 16, 7, 100, f);
  f.mb[8] = inactive();
  u.poll(200, f);
  CHECK(u.tx_done() == 1);
  TxTracker w;                                             // already sent: no mailbox holds it
  FakeMb g;
  w.on_written(8, 8, 16, 9, 100, g);
  CHECK(w.valid());
  CHECK(w.pending() == 0);
  CHECK(w.tx_done() == 1);
  TxTracker x;                                             // range the tracker cannot read: invalid
  x.on_written(8, 16, 16, 9, 100, g);
  CHECK_FALSE(x.valid());
}
