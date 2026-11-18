/*****************************************************************************************
 *  imu_capture.h — the Platform IMU capture ring, its anchors and the frozen-capture image.
 *
 *  Pure logic with no Arduino dependency, so tests/firmware/native/ runs the shipped code.
 *  Contract: ros_ws/docs/platform_imu_protocol.md sections 3 and 5. Every number comes from
 *  the generated protocol_config.h (PlatformImu::*), none is repeated here.
 *
 *  STORAGE IS INJECTED. The sketch hands in a sample buffer (6 x int16 per sample, placed in
 *  DMAMEM) and an anchor array; a test hands in small ones. No heap, no constructor with a
 *  side effect, no global.
 *
 *  A capture is the ring's retained samples, oldest first, plus the anchors and a header,
 *  read as one address space of 6-byte words (read_word). While FROZEN nothing is pushed,
 *  so a reader sees a stable image. The payload CRC-32 is computed in bounded slices
 *  (crc_step) so no single loop() pass is held for long.
 *****************************************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "protocol_config.h"

namespace ImuCapture {

struct Anchor {
  uint32_t index;     // samples appended since the capture began, after the drain it belongs to
  uint32_t local_us;  // low 32 bits of micros64() just before that drain's FIFO_COUNT read
  uint64_t wall_us;   // same instant on the synced wall clock
  uint8_t flags;      // PlatformImuAnchorFlag
};

// What the sketch knows about the chip and the board at freeze time (header fields the ring cannot know).
struct HeaderInfo {
  uint64_t freeze_wall_us = 0;
  uint32_t freeze_local_us = 0;
  uint16_t odr_hz = 0;
  uint16_t accel_lsb_per_g = 0;
  uint16_t gyro_lsb_per_dps_x10 = 0;
  uint8_t reg_gyro_accel_config0 = 0;
  uint8_t reg_accel_config0 = 0;
  uint8_t reg_gyro_config0 = 0;
  uint8_t i2c_clock_100khz = 0;
  int16_t temp_start_raw = 0;
  int16_t temp_freeze_raw = 0;
  uint16_t fifo_overflows = 0;
  uint16_t i2c_errors = 0;
  uint16_t sync_peak_us = 0;
  uint16_t fw_version = 0;
  uint16_t extra_flags = 0;  // PlatformImuCaptureFlag bits the ring cannot know: SENSOR_ERROR, NEVER_SYNCED, I2C_400K
};

inline void put16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
inline void put32(uint8_t* p, uint32_t v) { put16(p, uint16_t(v)); put16(p + 2, uint16_t(v >> 16)); }
inline void put64(uint8_t* p, uint64_t v) { put32(p, uint32_t(v)); put32(p + 4, uint32_t(v >> 32)); }

// CRC-32/IEEE (zlib.crc32), nibble table: no 1 KB table in RAM, no static initialiser.
inline uint32_t crc32_update(uint32_t crc, const uint8_t* d, size_t n) {
  static constexpr uint32_t T[16] = {
      0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
      0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu, 0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu};
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    crc = (crc >> 4) ^ T[crc & 15u];
    crc = (crc >> 4) ^ T[crc & 15u];
  }
  return crc;
}
inline uint32_t crc32(const uint8_t* d, size_t n) { return ~crc32_update(0xFFFFFFFFu, d, n); }

class Capture {
 public:
  static constexpr uint32_t WORD = PlatformImu::WORD_BYTES;
  static constexpr uint32_t HEADER_WORDS = PlatformImu::HEADER_BYTES / PlatformImu::WORD_BYTES;

  // buf: cap_samples x 6 int16. anchors: anchor_cap entries. boot_id nonzero random.
  void init(int16_t* buf, uint32_t cap_samples, Anchor* anchors, uint32_t anchor_cap, uint32_t boot_id) {
    buf_ = buf; cap_ = cap_samples; anchors_ = anchors; acap_ = anchor_cap; boot_id_ = boot_id;
    uint16_t lo = uint16_t(boot_id);
    next_id_ = lo ? lo : 1;
    capture_id_ = 0;
    reset();
  }

  // Start a new capture (boot, ENABLE from DISABLED, RESUME). Discards everything retained.
  void reset() {
    head_ = 0; count_ = 0; appended_ = 0; a_head_ = 0; a_count_ = 0;
    frozen_ = false; ready_ = false; n_ = 0; n_anchors_ = 0; crc_pos_ = 0; crc_ = 0xFFFFFFFFu;
    capture_id_ = 0;
  }

  bool frozen() const { return frozen_; }
  bool header_ready() const { return ready_; }   // payload CRC done and header built
  uint32_t n_samples() const { return frozen_ ? n_ : count_; }
  uint32_t appended() const { return appended_; }
  uint32_t capacity() const { return cap_; }
  uint16_t capture_id() const { return capture_id_; }
  uint32_t n_anchors() const { return frozen_ ? n_anchors_ : a_count_; }
  bool wrapped() const { return appended_ > cap_; }

  // Append one sample (ax ay az gx gy gz). Refused while frozen.
  bool push(const int16_t* six) {
    if (frozen_ || !buf_) return false;
    std::memcpy(buf_ + size_t(head_) * 6, six, 12);
    head_ = (head_ + 1 == cap_) ? 0 : head_ + 1;
    if (count_ < cap_) count_++;
    appended_++;
    return true;
  }

  // Record an anchor for the drain just pushed (index = samples appended so far). Refused while frozen.
  bool add_anchor(uint32_t local_us, uint64_t wall_us, uint8_t flags) {
    if (frozen_ || !acap_) return false;
    uint32_t slot = (a_head_ + a_count_) % acap_;
    if (a_count_ == acap_) { a_head_ = (a_head_ + 1) % acap_; slot = (a_head_ + a_count_ - 1) % acap_; }
    else a_count_++;
    anchors_[slot] = Anchor{appended_, local_us, wall_us, flags};
    return true;
  }

  // Freeze: snapshot the retained samples and anchors and start the payload CRC. False if empty or already frozen.
  bool begin_freeze(uint8_t reason) {
    if (frozen_ || count_ == 0) return false;
    reason_ = reason;
    n_ = count_;
    first_index_ = appended_ - count_;
    start_ = (head_ + cap_ - count_) % cap_;
    // Anchors whose sample (index - 1) is no longer retained are dropped.
    uint32_t skip = 0;
    while (skip < a_count_ && anchor_at(skip).index <= first_index_) skip++;
    a_first_ = skip;
    n_anchors_ = a_count_ - skip;
    capture_id_ = next_id_;
    next_id_ = uint16_t(next_id_ + 1);
    if (next_id_ == 0) next_id_ = 1;
    crc_ = 0xFFFFFFFFu;
    crc_pos_ = 0;
    ready_ = false;
    frozen_ = true;
    return true;
  }

  uint32_t payload_words() const { return 3u * n_anchors_ + 2u * n_; }

  // Advance the payload CRC by at most max_bytes (rounded to whole words). True once it is complete.
  bool crc_step(uint32_t max_bytes) {
    if (!frozen_) return false;
    uint32_t total = payload_words();
    uint32_t words = max_bytes / WORD;
    if (words == 0) words = 1;
    uint8_t w[WORD];
    while (crc_pos_ < total && words--) {
      payload_word(crc_pos_, w);
      crc_ = crc32_update(crc_, w, WORD);
      crc_pos_++;
    }
    return crc_pos_ >= total;
  }
  bool crc_done() const { return frozen_ && crc_pos_ >= payload_words(); }
  uint32_t payload_crc() const { return ~crc_; }

  // Build the 66-byte header once the CRC is done. Nothing is readable at the header seqs before this.
  bool finalize(const HeaderInfo& h) {
    if (!crc_done()) return false;
    std::memset(header_, 0, sizeof header_);
    uint16_t flags = h.extra_flags;
    if (wrapped()) flags |= PlatformImuCaptureFlag::WRAPPED;
    for (uint32_t i = 0; i < n_anchors_; i++)
      if (anchor_at(a_first_ + i).flags & PlatformImuAnchorFlag::DISCONTINUITY) flags |= PlatformImuCaptureFlag::DISCONTINUITY;
    uint8_t* p = header_;
    p[0] = uint8_t(PlatformImu::HEADER_FORMAT_VERSION);
    p[1] = reason_;
    put16(p + 2, capture_id_);
    put32(p + 4, boot_id_);
    put32(p + 8, n_);
    put32(p + 12, first_index_);
    put16(p + 16, uint16_t(n_anchors_));
    put16(p + 18, flags);
    put64(p + 20, h.freeze_wall_us);
    put32(p + 28, h.freeze_local_us);
    put16(p + 32, h.odr_hz);
    put16(p + 34, h.accel_lsb_per_g);
    put16(p + 36, h.gyro_lsb_per_dps_x10);
    p[38] = h.reg_gyro_accel_config0;
    p[39] = h.reg_accel_config0;
    p[40] = h.reg_gyro_config0;
    p[41] = h.i2c_clock_100khz;
    put16(p + 42, uint16_t(h.temp_start_raw));
    put16(p + 44, uint16_t(h.temp_freeze_raw));
    put16(p + 46, h.fifo_overflows);
    put16(p + 48, h.i2c_errors);
    put16(p + 50, h.sync_peak_us);
    put16(p + 52, h.fw_version);
    put32(p + 54, payload_crc());
    put32(p + 58, 0);
    put32(p + 62, crc32(p, 62));
    ready_ = true;
    return true;
  }
  const uint8_t* header() const { return header_; }

  // Exclusive end seq of the valid part of the region containing seq; 0 if seq is in no valid region.
  uint32_t region_end(uint32_t seq) const {
    if (!frozen_) return 0;
    if (seq < 2u * n_) return 2u * n_;
    uint32_t ab = PlatformImu::ANCHOR_SEQ_BASE;
    if (seq >= ab && seq < ab + 3u * n_anchors_) return ab + 3u * n_anchors_;
    uint32_t hb = PlatformImu::HEADER_SEQ_BASE;
    if (ready_ && seq >= hb && seq < hb + HEADER_WORDS) return hb + HEADER_WORDS;
    return 0;
  }
  // A READ must lie wholly inside one region's valid part.
  bool range_valid(uint32_t first, uint32_t count) const {
    if (count == 0) return false;
    uint32_t end = region_end(first);
    return end != 0 && first + count <= end;
  }

  // The 6-byte word at seq; false if seq is not valid. Anchors are the 18-byte records split in three.
  bool read_word(uint32_t seq, uint8_t* out6) const {
    if (region_end(seq) == 0) return false;
    if (seq < 2u * n_) { sample_word(seq, out6); return true; }
    uint32_t hb = PlatformImu::HEADER_SEQ_BASE;
    if (seq >= hb) { std::memcpy(out6, header_ + (seq - hb) * WORD, WORD); return true; }
    uint32_t r = seq - PlatformImu::ANCHOR_SEQ_BASE;
    uint8_t rec[PlatformImu::ANCHOR_BYTES];
    anchor_bytes(a_first_ + r / 3u, rec);
    std::memcpy(out6, rec + (r % 3u) * WORD, WORD);
    return true;
  }

 private:
  const Anchor& anchor_at(uint32_t i) const { return anchors_[(a_head_ + i) % acap_]; }
  void anchor_bytes(uint32_t i, uint8_t* rec) const {
    const Anchor& a = anchor_at(i);
    put32(rec, a.index);
    put32(rec + 4, a.local_us);
    put64(rec + 8, a.wall_us);
    rec[16] = a.flags;
    rec[17] = 0;
  }
  void sample_word(uint32_t w, uint8_t* out) const {
    uint32_t k = w >> 1;
    const int16_t* s = buf_ + size_t((start_ + k) % cap_) * 6 + (w & 1u) * 3;
    for (int i = 0; i < 3; i++) put16(out + 2 * i, uint16_t(s[i]));
  }
  // Payload word p: the anchor words, then the sample words, in seq order.
  void payload_word(uint32_t p, uint8_t* out) const {
    if (p < 3u * n_anchors_) read_word(PlatformImu::ANCHOR_SEQ_BASE + p, out);
    else sample_word(p - 3u * n_anchors_, out);
  }

  int16_t* buf_ = nullptr;
  Anchor* anchors_ = nullptr;
  uint32_t cap_ = 0, acap_ = 0, boot_id_ = 0;
  uint32_t head_ = 0, count_ = 0, appended_ = 0;
  uint32_t a_head_ = 0, a_count_ = 0;
  bool frozen_ = false, ready_ = false;
  uint16_t next_id_ = 1, capture_id_ = 0;
  uint8_t reason_ = 0;
  // frozen snapshot
  uint32_t n_ = 0, n_anchors_ = 0, first_index_ = 0, start_ = 0, a_first_ = 0;
  uint32_t crc_pos_ = 0;
  uint32_t crc_ = 0xFFFFFFFFu;
  uint8_t header_[PlatformImu::HEADER_BYTES] = {};
};

}  // namespace ImuCapture
