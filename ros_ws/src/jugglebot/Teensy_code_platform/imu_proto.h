/*****************************************************************************************
 *  imu_proto.h — Platform IMU frames, state machine and READ pacing.
 *
 *  Pure logic with no Arduino dependency, driven by an injected clock (micros as uint32,
 *  wrap-safe). Contract: ros_ws/docs/platform_imu_protocol.md sections 2 and 4. The sketch
 *  owns the CAN mailboxes, the chip and the clock; this owns what a command means.
 *
 *  Per loop() pass the sketch does, in this order:
 *    1. decode an IMU_CMD  -> parse_cmd, Engine::handle, send the Reply
 *    2. Engine::service_freeze (one bounded CRC slice) and send the FREEZE reply when it returns true
 *    3. Engine::peek_data -> try to place the frame in a free mailbox -> Engine::commit_data only if placed
 *    4. Engine::status_due -> pack_status
 *  Internal transitions (chip ready, I2C error, back-off, FwUpdate) are the on_* calls.
 *****************************************************************************************/
#pragma once

#include <cstdint>
#include <cstring>

#include "imu_capture.h"
#include "protocol_config.h"

namespace ImuProto {

using ImuCapture::put16;
using ImuCapture::put32;

struct Frame {
  uint8_t d[8];
};

struct Cmd {
  bool valid = false;  // false: unknown op or arguments that cannot be parsed (reply ERR_BAD_ARGS)
  uint8_t op = 0;
  uint8_t reason = 0;
  uint16_t capture_id = 0;
  uint16_t first_seq = 0;
  uint16_t count = 0;
  uint8_t gap_10us = 0;
};

struct Reply {
  uint8_t op = 0;
  uint8_t status = 0;
  uint16_t capture_id = 0;
  uint32_t detail = 0;
};

struct Status {
  uint8_t state = 0;
  uint8_t flags = 0;
  int8_t temp_c = -128;
  uint8_t fifo_overflows = 0;
  uint8_t i2c_errors = 0;
  uint8_t sync_peak = 255;
  uint8_t fill_s = 0;
  uint8_t boot = 0;
};

inline uint16_t get16(const uint8_t* p) { return uint16_t(p[0] | (p[1] << 8)); }

inline Cmd parse_cmd(const uint8_t* d, uint8_t len) {
  Cmd c;
  if (len < 1) return c;
  c.op = d[0];
  using namespace PlatformImuOp;
  switch (c.op) {
    case ENABLE: case DISABLE: case ABORT:
      c.valid = true;
      break;
    case FREEZE:
      if (len < 2) break;
      c.reason = d[1];
      c.valid = c.reason >= PlatformImuFreezeReason::ATTEMPT_END && c.reason <= PlatformImuFreezeReason::OPERATOR;
      break;
    case RESUME:
      if (len < 3) break;
      c.capture_id = get16(d + 1);
      c.valid = true;
      break;
    case READ:
      if (len < 8) break;
      c.capture_id = get16(d + 1);
      c.first_seq = get16(d + 3);
      c.count = get16(d + 5);
      c.gap_10us = d[7];
      c.valid = c.count >= 1 && c.count <= PlatformImu::READ_MAX_WORDS;
      break;
    default:
      break;
  }
  return c;
}

inline Frame pack_reply(const Reply& r) {
  Frame f{};
  f.d[0] = r.op;
  f.d[1] = r.status;
  put16(f.d + 2, r.capture_id);
  put32(f.d + 4, r.detail);
  return f;
}
inline Reply unpack_reply(const Frame& f) {
  Reply r;
  r.op = f.d[0];
  r.status = f.d[1];
  r.capture_id = get16(f.d + 2);
  r.detail = uint32_t(get16(f.d + 4)) | (uint32_t(get16(f.d + 6)) << 16);
  return r;
}
inline Frame pack_status(const Status& s) {
  Frame f{};
  f.d[0] = s.state; f.d[1] = s.flags; f.d[2] = uint8_t(s.temp_c); f.d[3] = s.fifo_overflows;
  f.d[4] = s.i2c_errors; f.d[5] = s.sync_peak; f.d[6] = s.fill_s; f.d[7] = s.boot;
  return f;
}
inline Frame pack_data(uint16_t seq, const uint8_t* word6) {
  Frame f{};
  put16(f.d, seq);
  std::memcpy(f.d + 2, word6, 6);
  return f;
}

inline uint8_t sat_u8(uint32_t v) { return v > 255u ? 255u : uint8_t(v); }
// IMU_STATUS fill_s: retained samples / nominal ODR, whole seconds, saturating.
inline uint8_t fill_seconds(uint32_t retained, uint32_t odr_hz) { return odr_hz ? sat_u8(retained / odr_hz) : 0; }

class Engine {
 public:
  static constexpr uint32_t BACKOFF_MS = 1000;
  static constexpr uint32_t CRC_SLICE_BYTES = 8192;  // one slice per loop() pass

  // Boots into PROBING (enabled), like the board.
  void init(ImuCapture::Capture* cap) {
    cap_ = cap;
    state_ = PlatformImuState::PROBING;
    block_active_ = false;
    pending_freeze_reply_ = false;
    disc_pending_ = false;
    resume_check_ = false;
    have_sent_ = false;
    last_status_ms_ = 0;
    status_started_ = false;
  }

  uint8_t state() const { return state_; }
  bool block_active() const { return block_active_; }
  bool freeze_pending() const { return pending_freeze_reply_; }
  // True once since the last call if samples were lost (I2C error, overflow): the next anchor is DISCONTINUITY.
  bool take_discontinuity() { bool d = disc_pending_; disc_pending_ = false; return d; }
  void note_overflow() { disc_pending_ = true; }
  // True once since the last call after a RESUME: the sketch flushes the chip FIFO and re-checks WHO_AM_I.
  bool take_resume_check() { bool r = resume_check_; resume_check_ = false; return r; }
  // The sketch may push samples and take anchors only while this is true: RUNNING, and the post-RESUME FIFO
  // flush has been serviced (take_resume_check called). The chip kept streaming while FROZEN, so its FIFO holds
  // up to 128 stale packets that must not enter the new capture.
  bool sampling() const { return state_ == PlatformImuState::RUNNING && !resume_check_; }

  /* ----------------------------- commands ----------------------------- */

  // One reply per command. FREEZE answers OK later, through service_freeze, once its CRC is done.
  // For a FREEZE that was accepted, `deferred` is set and the returned Reply must NOT be sent.
  Reply handle(const Cmd& c, const ImuCapture::HeaderInfo& info, bool* deferred) {
    using namespace PlatformImuStatus;
    using namespace PlatformImuOp;
    *deferred = false;
    Reply r;
    r.op = c.op;
    // The frozen capture's id (0 when none): the spec's rule for every reply, refusals included. DISABLE
    // overrides it below because it discards the capture; FW_HALTED has already reset it.
    r.capture_id = cap_->frozen() ? cap_->capture_id() : 0;
    if (!c.valid) return fail(r, ERR_BAD_ARGS);
    if (state_ == PlatformImuState::FW_HALTED) return fail(r, ERR_FW_UPDATE);
    const uint8_t s = state_;
    const bool frozenish = s == PlatformImuState::FROZEN || s == PlatformImuState::UPLOADING;
    if (s == PlatformImuState::DISABLED) {
      if (c.op == ENABLE) { reset_capture(); state_ = PlatformImuState::PROBING; return r; }
      if (c.op == DISABLE || c.op == ABORT) return r;
      return fail(r, ERR_DISABLED);
    }
    switch (c.op) {
      case ENABLE:
        return r;
      case DISABLE:
        cancel_block();
        pending_freeze_reply_ = false;
        reset_capture();
        r.capture_id = 0;  // the capture is discarded
        state_ = PlatformImuState::DISABLED;
        return r;
      case ABORT:
        if (s == PlatformImuState::UPLOADING) { cancel_block(); state_ = PlatformImuState::FROZEN; }
        return r;
      case FREEZE:
        if (frozenish) {
          r.status = ALREADY_FROZEN;
          r.detail = cap_->n_samples();
          return r;
        }
        if (cap_->n_samples() == 0) return fail(r, ERR_EMPTY);
        pending_info_ = info;
        if (s != PlatformImuState::RUNNING) pending_info_.extra_flags |= PlatformImuCaptureFlag::SENSOR_ERROR;
        cap_->begin_freeze(c.reason);
        state_ = PlatformImuState::FROZEN;
        pending_freeze_reply_ = true;
        *deferred = true;
        return r;
      case RESUME:
        if (!frozenish) return fail(r, ERR_NOT_FROZEN);
        r.capture_id = cap_->capture_id();
        if (c.capture_id != cap_->capture_id()) return fail(r, ERR_WRONG_CAPTURE);
        cancel_block();
        pending_freeze_reply_ = false;
        reset_capture();
        state_ = PlatformImuState::RUNNING;
        resume_check_ = true;  // sampling() stays false until the sketch has flushed the chip FIFO
        return r;
      case READ:
        if (!frozenish) return fail(r, ERR_NOT_FROZEN);
        r.capture_id = cap_->capture_id();
        if (c.capture_id != cap_->capture_id()) return fail(r, ERR_WRONG_CAPTURE);
        if (!cap_->range_valid(c.first_seq, c.count)) {
          r.status = ERR_OUT_OF_RANGE;
          r.detail = uint32_t(c.first_seq) | (cap_->region_end(c.first_seq) << 16);
          return r;
        }
        block_active_ = true;
        next_seq_ = c.first_seq;
        remaining_ = c.count;
        gap_us_ = gap_for(c.gap_10us);
        state_ = PlatformImuState::UPLOADING;
        r.detail = uint32_t(c.first_seq) | (uint32_t(c.count) << 16);
        return r;
      default:
        return fail(r, ERR_BAD_ARGS);
    }
  }

  /* --------------------- freeze CRC and the READ block --------------------- */

  // One bounded CRC slice. Returns true when the FREEZE reply is ready in *out (send it once).
  bool service_freeze(Reply* out) {
    if (!pending_freeze_reply_) return false;
    if (!cap_->crc_step(CRC_SLICE_BYTES)) return false;
    cap_->finalize(pending_info_);
    pending_freeze_reply_ = false;
    out->op = PlatformImuOp::FREEZE;
    out->status = PlatformImuStatus::OK;
    out->capture_id = cap_->capture_id();
    out->detail = cap_->n_samples();
    return true;
  }

  // Gap between DATA frames: frame_gap_10us x 10 us, raised to read_min_gap_10us.
  static uint32_t gap_for(uint8_t gap_10us) {
    uint32_t g = gap_10us < PlatformImu::READ_MIN_GAP_10US ? PlatformImu::READ_MIN_GAP_10US : gap_10us;
    return g * 10u;
  }

  // The next DATA frame, if one is due now. Does not advance: call commit_data once it is in a mailbox.
  bool peek_data(uint32_t now_us, Frame* out) const {
    if (!block_active_) return false;
    if (!gap_elapsed(now_us)) return false;
    uint8_t w[6];
    if (!cap_->read_word(next_seq_, w)) return false;
    *out = pack_data(uint16_t(next_seq_), w);
    return true;
  }
  void commit_data(uint32_t now_us) {
    if (!block_active_) return;
    next_seq_++;
    remaining_--;
    last_sent_us_ = now_us;  // kept across blocks: the gap floor also separates the last frame of one block from the first of the next
    have_sent_ = true;
    if (remaining_ == 0) {
      block_active_ = false;
      if (state_ == PlatformImuState::UPLOADING) state_ = PlatformImuState::FROZEN;
    }
  }
  // Microseconds until the next DATA frame is due (0 if due or none pending).
  uint32_t data_wait_us(uint32_t now_us) const {
    if (!block_active_ || !have_sent_) return 0;
    uint32_t age = now_us - last_sent_us_;
    return age >= gap_us_ ? 0u : gap_us_ - age;
  }

  /* ------------------------- internal transitions ------------------------- */

  void on_chip_ready() { if (state_ == PlatformImuState::PROBING) state_ = PlatformImuState::RUNNING; }
  void on_i2c_error(uint32_t now_ms) {
    if (state_ == PlatformImuState::RUNNING) {
      state_ = PlatformImuState::BACKOFF;
      backoff_until_ms_ = now_ms + BACKOFF_MS;
      disc_pending_ = true;
    }
  }
  // BACKOFF to PROBING after the back-off. Call every pass.
  void tick(uint32_t now_ms) {
    if (state_ == PlatformImuState::BACKOFF && int32_t(now_ms - backoff_until_ms_) >= 0)
      state_ = PlatformImuState::PROBING;
  }
  // After RESUME the chip may not answer: PROBING instead of RUNNING.
  void on_resume_chip_failed() { if (state_ == PlatformImuState::RUNNING) state_ = PlatformImuState::PROBING; }
  void on_fw_update_begin() {
    cancel_block();
    pending_freeze_reply_ = false;
    reset_capture();
    state_ = PlatformImuState::FW_HALTED;
  }
  void on_fw_update_abandoned() { if (state_ == PlatformImuState::FW_HALTED) state_ = PlatformImuState::PROBING; }

  /* --------------------------------- status --------------------------------- */

  // True once per STATUS_PERIOD_MS (the first call is due at once).
  bool status_due(uint32_t now_ms) {
    if (!status_started_ || uint32_t(now_ms - last_status_ms_) >= PlatformImu::STATUS_PERIOD_MS) {
      status_started_ = true;
      last_status_ms_ = now_ms;
      return true;
    }
    return false;
  }

 private:
  static Reply& fail(Reply& r, uint8_t status) { r.status = status; return r; }
  void cancel_block() { block_active_ = false; remaining_ = 0; }
  // A new capture begins: nothing from the old one (a pending resume flush, a pending discontinuity) carries over.
  void reset_capture() { cap_->reset(); resume_check_ = false; disc_pending_ = false; }
  // Wrap-safe: the next frame is due once gap_us_ has passed since the last frame sent (any block). An age that
  // is negative as an int32 (clock read just before the stamp) counts as elapsed; so does the first frame ever.
  bool gap_elapsed(uint32_t now_us) const {
    if (!have_sent_) return true;
    uint32_t age = now_us - last_sent_us_;
    return age >= gap_us_;
  }

  ImuCapture::Capture* cap_ = nullptr;
  ImuCapture::HeaderInfo pending_info_;
  uint8_t state_ = PlatformImuState::PROBING;
  bool pending_freeze_reply_ = false, resume_check_ = false, disc_pending_ = false;
  bool block_active_ = false, have_sent_ = false;
  uint32_t next_seq_ = 0, remaining_ = 0, gap_us_ = 0, last_sent_us_ = 0;
  uint32_t backoff_until_ms_ = 0, last_status_ms_ = 0;
  bool status_started_ = false;
};

}  // namespace ImuProto
