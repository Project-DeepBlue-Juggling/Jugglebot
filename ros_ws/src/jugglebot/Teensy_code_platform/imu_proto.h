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
 *
 *  FW 13: a READ contiguous with the active block extends it (READ_EXTENDED); DATA frames
 *  follow a fixed schedule (due += gap) with at most MAX_CATCHUP_FRAMES of catch-up; SET_ODR
 *  selects the capture rate (1 or 2 kHz), which this engine holds (odr_hz()).
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
  uint16_t odr_hz = 0;  // SET_ODR: the requested rate (1000 or FAST_ODR_HZ)
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
    case SET_ODR:  // [odr_khz u8]: 1 or 2 kHz
      if (len < 2) break;
      c.odr_hz = uint16_t(d[1]) * 1000u;
      c.valid = c.odr_hz == PlatformImu::NOMINAL_ODR_HZ || c.odr_hz == PlatformImu::FAST_ODR_HZ;
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
  /*  FIXED-SCHEDULE PACING (FW 13).  Frame k+1 is due one gap after frame k's DUE time, not its
   *  send time, so loop() jitter no longer stretches the pace (measured 0.83 ms for 0.80 asked).
   *  A frame sent late is late against the schedule by `lag`; lag is clamped to this many gaps,
   *  so after a stall of any length at most this many frames follow faster than the gap: in any
   *  window W a block sends at most W / gap + 1 + MAX_CATCHUP_FRAMES frames. */
  static constexpr uint32_t MAX_CATCHUP_FRAMES = 2;

  // Boots into PROBING (enabled), like the board, at `odr_hz` (1000 or FAST_ODR_HZ).
  void init(ImuCapture::Capture* cap, uint16_t odr_hz = uint16_t(PlatformImu::NOMINAL_ODR_HZ)) {
    cap_ = cap;
    odr_hz_ = odr_hz;
    rate_changed_ = false;
    fresh_block_ = false;
    due_in_us_ = 0;
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
  // The selected capture rate, Hz: the rate of every sample in the current capture (SET_ODR starts a new one).
  uint16_t odr_hz() const { return odr_hz_; }
  // The fastest rate SET_ODR accepts (the sketch lowers it to NOMINAL_ODR_HZ on a 400 kHz I2C build).
  void set_max_odr_hz(uint16_t hz) { max_odr_hz_ = hz; }
  // True once since the last call after SET_ODR changed the rate: the sketch re-probes the chip at it.
  bool take_rate_change() { bool c = rate_changed_; rate_changed_ = false; return c; }
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
    if (c.op == SET_ODR) {
      /*  SET_ODR (FW 13).  Same rate: OK, nothing changes (an RPC retry is idempotent).  FROZEN or
       *  UPLOADING: ERR_FROZEN, so a rate switch never destroys a capture not yet uploaded.
       *  Otherwise the rate is stored and the capture discarded (one capture is always one
       *  rate); RUNNING re-probes at once, PROBING and BACKOFF keep their probe schedule, and
       *  DISABLED keeps the rate for the next ENABLE.  Detail: the selected rate, Hz. */
      if (c.odr_hz > max_odr_hz_) return fail(r, ERR_BAD_ARGS);
      if (c.odr_hz == odr_hz_) { r.detail = odr_hz_; return r; }
      if (frozenish) return fail(r, ERR_FROZEN);
      odr_hz_ = c.odr_hz;
      r.detail = odr_hz_;
      if (s == PlatformImuState::DISABLED) return r;
      reset_capture();
      rate_changed_ = true;
      if (s == PlatformImuState::RUNNING) state_ = PlatformImuState::PROBING;
      return r;
    }
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
        r.detail = uint32_t(c.first_seq) | (uint32_t(c.count) << 16);
        gap_us_ = gap_for(c.gap_10us);
        if (block_active_ && uint32_t(c.first_seq) == next_seq_ + remaining_) {
          /*  READ CHAINING (FW 13): contiguous with the active block, so it is appended and the
           *  pace schedule carries on.  Regions are never contiguous, so the block stays in one
           *  region; a retried READ is not contiguous (its first_seq is inside the block) and
           *  replaces the block below, which re-sends but never loses a word. */
          remaining_ += c.count;
          r.status = READ_EXTENDED;
          return r;
        }
        block_active_ = true;
        fresh_block_ = true;
        next_seq_ = c.first_seq;
        remaining_ = c.count;
        state_ = PlatformImuState::UPLOADING;
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
    /*  The schedule: lag = how late this frame is against its due time, clamped to
     *  MAX_CATCHUP_FRAMES gaps; the next frame is due one gap after this one's due time.  The
     *  first frame of a fresh block starts the schedule (lag 0), so the idle time between
     *  blocks never turns into a burst.  Everything is an offset from the last send, wrap-safe. */
    int64_t lag = 0;
    if (have_sent_ && !fresh_block_) {
      lag = int64_t(uint32_t(now_us - last_sent_us_)) - int64_t(due_in_us_);
      const int64_t cap = int64_t(MAX_CATCHUP_FRAMES) * int64_t(gap_us_);
      if (lag < 0) lag = 0;
      if (lag > cap) lag = cap;
    }
    due_in_us_ = int32_t(int64_t(gap_us_) - lag);
    fresh_block_ = false;
    last_sent_us_ = now_us;  // kept across blocks: a fresh block's first frame still waits a full gap after it
    have_sent_ = true;
    if (remaining_ == 0) {
      block_active_ = false;
      if (state_ == PlatformImuState::UPLOADING) state_ = PlatformImuState::FROZEN;
    }
  }
  // Microseconds until the next DATA frame is due (0 if due or none pending).
  uint32_t data_wait_us(uint32_t now_us) const {
    if (!block_active_ || !have_sent_) return 0;
    const int32_t due = due_after_last();
    if (due <= 0) return 0;
    uint32_t age = now_us - last_sent_us_;
    return age >= uint32_t(due) ? 0u : uint32_t(due) - age;
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
  // Microseconds after the last frame sent at which the next is due: the schedule's offset inside a block (may
  // be <= 0 while catching up), a full gap for the first frame of a fresh block (the floor between blocks).
  int32_t due_after_last() const { return fresh_block_ ? int32_t(gap_us_) : due_in_us_; }
  // Wrap-safe: an offset from the last frame sent (any block), as before FW 13; the first frame ever is due at once.
  bool gap_elapsed(uint32_t now_us) const {
    if (!have_sent_) return true;
    const int32_t due = due_after_last();
    if (due <= 0) return true;
    uint32_t age = now_us - last_sent_us_;
    return age >= uint32_t(due);
  }

  ImuCapture::Capture* cap_ = nullptr;
  ImuCapture::HeaderInfo pending_info_;
  uint8_t state_ = PlatformImuState::PROBING;
  bool pending_freeze_reply_ = false, resume_check_ = false, disc_pending_ = false;
  bool block_active_ = false, have_sent_ = false, fresh_block_ = false, rate_changed_ = false;
  uint32_t next_seq_ = 0, remaining_ = 0, gap_us_ = 0, last_sent_us_ = 0;
  int32_t due_in_us_ = 0;  // next frame due this long after the last send (<= 0: due at once, catching up)
  uint16_t odr_hz_ = uint16_t(PlatformImu::NOMINAL_ODR_HZ), max_odr_hz_ = uint16_t(PlatformImu::FAST_ODR_HZ);
  uint32_t backoff_until_ms_ = 0, last_status_ms_ = 0;
  bool status_started_ = false;
};

}  // namespace ImuProto
