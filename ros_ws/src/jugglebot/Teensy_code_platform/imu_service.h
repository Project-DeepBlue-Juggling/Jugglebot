/*****************************************************************************************
 *  imu_service.h — the Platform IMU's per-loop() glue (FW 11-12, IDEA-20 P2b).
 *
 *  Pure logic with no Arduino dependency, so tests/firmware/native/ runs the shipped code
 *  (test_platform_imu_service.cpp). It wires imu_proto.h's Engine and imu_capture.h's ring
 *  to the chip and the CAN mailboxes, which the sketch hands in as two small adapters:
 *
 *    Io  void init_start();                       arm the driver's init (no I2C)
 *        int  init_step(uint32_t now_us);         ONE I2C transaction: INIT_PENDING, 0 (ready) or < 0
 *        int  fifo_count(bool* overflow);         ONE transaction: packets in the chip FIFO, or < 0
 *        int  read_burst(Pkt* out, int n);        ONE transaction (n <= BURST_PKTS): packets parsed, or < 0
 *        int  flush();                            ONE transaction (+200 us pause): 0 or < 0
 *        int  check_who();                        ONE transaction: 0 if WHO_AM_I answers right, else < 0
 *        void bus_recover();                      I2C bus clear (9 SCL pulses + STOP on GPIO) and an I2C
 *                                                 controller reset; bounded (~0.12 ms), no device transaction
 *        void stamp(Stamp* s);                    clocks only: micros64() and the synced wall clock together
 *        void header_info(ImuCapture::HeaderInfo* h);  the chip's constant header fields (no I2C)
 *    Tx  bool place(uint32_t id, const uint8_t d[8]);
 *                                                 true only if the frame went straight into a free TX
 *                                                 mailbox (tx_may_place decided it), never the SW queue
 *
 *  THE ONE RULE THIS FILE EXISTS FOR.  The IMU is strictly optional: this board's USB port is
 *  dead and the firmware-over-CAN receiver, which only runs while loop() keeps calling
 *  can1.events(), is its only way back in.  So one pass() does AT MOST ONE bounded step:
 *  one I2C transaction (an init step, a FIFO count, one burst of <= 8 packets, a flush or a
 *  WHO_AM_I), or one bus recovery (before the next transfer after any counted error), or one
 *  CRC slice, or one IMU_DATA frame; never two, never a wait.  A failed
 *  transaction is never retried in the same pass.  In RUNNING a failed FIFO transfer is
 *  retried on later passes, behind a bus recovery, up to RUNNING_MAX_RETRIES times in a row
 *  (THE RUNNING RETRY POLICY, drain_step); the next failure in a row goes to BACKOFF, and
 *  repeated probe failures back off from PROBE_MIN_GAP_MS to PROBE_MAX_GAP_MS.  For BOOT_SETTLE_MS after boot, and while a
 *  firmware-update session is open, nothing here touches the bus.  Contract: ros_ws/docs/platform_imu_protocol.md.
 *****************************************************************************************/
#pragma once

#include <cstdint>
#include <cstring>

#include "imu_proto.h"

#ifndef PLATFORM_IMU_DIAG
#define PLATFORM_IMU_DIAG 0   // 1 = the diagnostic build (Teensy_code_platform.ino, "IMU DIAGNOSTIC BUILD")
#endif

namespace ImuService {

/*  THE BOOT SETTLE (P2 audit B3).  For the first 10 s after boot pass() does nothing but drop
 *  IMU_CMD frames and watch for a firmware-update BEGIN: no I2C, no Wire set-up (the sketch
 *  starts Wire at the first probe), no engine work, no IMU frame.  If IMU code ever faults the
 *  board, the core reboots it 8 s later, so every reboot cycle has this IMU-free window in
 *  which a BEGIN can land; a BEGIN seen inside it keeps the IMU in FW_HALTED for the rest of
 *  that boot, even if the update is then abandoned. */
constexpr uint32_t BOOT_SETTLE_MS = 10000;
constexpr uint32_t DRAIN_PERIOD_US = 4000;   // FIFO_COUNT read cadence: ~4 packets per drain at 1 kHz
constexpr int      BURST_PKTS = 8;           // 8 x 16 B = 128 B, inside Wire's 136-byte BUFFER_LENGTH
constexpr uint32_t DRAIN_MAX_PKTS = 120;     // never read past what was counted, never near the 128-record FIFO
constexpr uint32_t PROBE_MIN_GAP_MS = 1000;  // probe back-off floor (also the spec's BACKOFF minimum)
constexpr uint32_t PROBE_MAX_GAP_MS = 60000; // and ceiling
constexpr uint32_t HEALTHY_RESET_MS = 10000; // this long RUNNING without a BACKOFF resets the back-off
/*  Consecutive failed FIFO transfers (count, burst or flush) RUNNING absorbs with a bus recovery
 *  and a retry; the next failure in a row, with no successful transfer between, goes to BACKOFF.
 *  2 (FW 12, 2026-11-20): the measured errors are isolated (one every 15-40 s), so a third in a
 *  row means the chip or the bus is really gone, and 3 failures + 2 recoveries take ~5 passes,
 *  far inside the chip FIFO's 128 ms at 1 kHz. */
constexpr uint32_t RUNNING_MAX_RETRIES = 2;
/*  TX mailboxes the IMU always leaves free for everyone else.  The sketch has 8 (setMaxMB(16),
 *  the RX FIFO and its filter table take MB 0-7).  After an IMU frame the non-IMU writers that
 *  can run before loop() next checks the mailboxes for a dispatch are the Hand fan status
 *  (this pass), the tilt reply, the traffic report and the Hand fan status (the next pass): 3,
 *  plus one for a Hand fan change in the pass after a periodic status.  So 4, and the IMU
 *  keeps at most 4 frames pending: at 111 us per frame on the wire still far above its
 *  1600 frames/s READ cap.  LoopService::Can::DISPATCH_MIN_FREE_TX is the dispatch side. */
constexpr uint32_t TX_RESERVE_MAILBOXES = 4;
constexpr uint8_t  CMD_SLOTS = 8;            // pending IMU_CMD frames (7 usable)
constexpr uint8_t  REPLY_SLOTS = 8;          // pending IMU_REPLY frames (7 usable)
constexpr int      INIT_PENDING = 1;         // icm42688::Driver::INIT_PENDING (the sketch static_asserts both)
constexpr int      ERR_BAD_PACKET = -5;      // icm42688::ERR_BAD_PACKET

/*  THE TX-MAILBOX RULE (spec section 4).  FlexCAN_T4's write() queues a frame in software when
 *  no mailbox is free, and its events() then writes that queued frame into EVERY free mailbox
 *  (FlexCAN_T4.tpp events(), the frame.mb == -1 loop): duplicates, and the rest of the queue
 *  dropped.  An IMU frame is therefore handed to write() only when the software queue is empty
 *  and more than TX_RESERVE_MAILBOXES mailboxes are free, so write() always lands it in a
 *  mailbox and the IMU can never take the last mailboxes the firmware-update, tilt, state and
 *  fan replies need.  The sketch evaluates this with the CAN interrupt masked. */
inline bool tx_may_place(uint32_t free_tx_mailboxes, uint32_t sw_tx_queued) {
  return sw_tx_queued == 0 && free_tx_mailboxes > TX_RESERVE_MAILBOXES;
}

/*  The back-off after `streak` consecutive failures (>= 1): 1, 2, 4 ... 32, then 60 s. */
inline uint32_t backoff_ms(uint32_t streak) {
  if (streak == 0) return 0;
  const uint32_t sh = streak - 1 > 6 ? 6 : streak - 1;
  const uint32_t d = PROBE_MIN_GAP_MS << sh;
  return d > PROBE_MAX_GAP_MS ? PROBE_MAX_GAP_MS : d;
}

/*  IMU_STATUS temp_c from the FIFO's 8-bit temperature (degC = raw / 2.07 + 25), rounded. */
inline int8_t temp_c_from_fifo(int8_t raw) {
  const int32_t num = int32_t(raw) * 100;
  const int32_t c = 25 + (num >= 0 ? (num + 103) / 207 : (num - 103) / 207);
  return int8_t(c > 127 ? 127 : (c < -127 ? -127 : c));
}
/*  Header temp fields are TEMP_DATA counts (1/132.48 degC); the FIFO byte is 1/2.07 degC, and
 *  132.48 / 2.07 = 64 exactly, so the FIFO byte x 64 is the TEMP_DATA value at 8-bit resolution
 *  (no extra register read). */
inline int16_t temp_data_from_fifo(int8_t raw) { return int16_t(int32_t(raw) * 64); }

struct Stamp {
  uint32_t local_us = 0;   // low 32 bits of micros64()
  uint64_t wall_us = 0;    // micros64() + the wall offset, same instant
  bool wall_valid = false; // a 0x7DD frame has set the offset
};

struct Pkt {
  int16_t s[6];  // ax ay az gx gy gz, chip LSB
  int8_t temp;   // FIFO temperature byte
};

/*  IMU_CMD frames, copied in by the CAN callback and nothing else.  Single producer
 *  (canSniff), single consumer (pass()).  Until loop()'s first can1.events() canSniff runs in
 *  FlexCAN's ISR, during setup(), when pass() never runs; after it canSniff runs inside
 *  events() in loop() context, so producer and consumer never overlap.  Full: the new frame is
 *  dropped (no reply; the host's RPC retry covers it) and counted. */
class CmdSlots {
 public:
  bool push(const uint8_t* d, uint8_t len) {
    const uint8_t next = uint8_t((head_ + 1) % CMD_SLOTS);
    if (next == tail_) { dropped_++; return false; }
    if (len > 8) len = 8;
    std::memset(buf_[head_], 0, 8);
    std::memcpy(buf_[head_], d, len);
    len_[head_] = len;
    head_ = next;
    return true;
  }
  bool pop(uint8_t out[8], uint8_t* len) {
    if (tail_ == head_) return false;
    std::memcpy(out, buf_[tail_], 8);
    *len = len_[tail_];
    tail_ = uint8_t((tail_ + 1) % CMD_SLOTS);
    return true;
  }
  uint32_t dropped() const { return dropped_; }

 private:
  volatile uint8_t head_ = 0, tail_ = 0;
  uint8_t buf_[CMD_SLOTS][8] = {};
  uint8_t len_[CMD_SLOTS] = {};
  uint32_t dropped_ = 0;
};

/*  Clock-sync quality, fed by the 0x7DD handler.  Integer only: during setup() that handler
 *  runs in FlexCAN's ISR.  `innovation_us` is the received wall time minus get_wall_time_us()
 *  before the IIR update; the first frame (the step) is not an innovation and sets SYNC_STEP. */
class SyncQuality {
 public:
  void note(bool first_frame, int64_t innovation_us) {
    frames_++;
    if (first_frame) { step_ = true; return; }
    uint32_t m;
    if (innovation_us >= 0) m = innovation_us > 0xFFFFFFFFll ? 0xFFFFFFFFu : uint32_t(innovation_us);
    else m = innovation_us < -0xFFFFFFFFll ? 0xFFFFFFFFu : uint32_t(-innovation_us);
    if (m > period_peak_) period_peak_ = m;
    if (m > capture_peak_) capture_peak_ = m;
  }
  /*  IMU_STATUS sync_peak (10 us counts, rounded up, 254 saturating, 255 = no frame) and the
   *  TIME_SYNCED flag for the period just ended; starts the next period. */
  void take_period(uint8_t* peak10, bool* synced) {
    *synced = frames_ != 0;
    if (!frames_) {
      *peak10 = 255;
    } else {
      const uint32_t c = period_peak_ / 10 + (period_peak_ % 10 ? 1u : 0u);
      *peak10 = c > 254 ? 254 : uint8_t(c);
    }
    frames_ = 0;
    period_peak_ = 0;
  }
  uint16_t capture_peak_us() const { return capture_peak_ > 0xFFFFu ? 0xFFFFu : uint16_t(capture_peak_); }
  void reset_capture() { capture_peak_ = 0; }
  bool take_step() { bool s = step_; step_ = false; return s; }

 private:
  volatile uint32_t frames_ = 0, period_peak_ = 0, capture_peak_ = 0;
  volatile bool step_ = false;
};

struct Counters {
  uint32_t fifo_overflows = 0, i2c_errors = 0, bad_packets = 0, probes = 0, tx_deferred = 0;
  uint32_t cmds_in_settle = 0;  // IMU_CMD frames dropped unanswered during the boot settle
  uint32_t recoveries = 0;      // bus clears + controller resets (one before the next transfer after an error)
  uint32_t retries = 0;         // RUNNING errors absorbed by the retry policy (no BACKOFF); each is also in i2c_errors
};

/*  DIAGNOSTIC BUILD (-DPLATFORM_IMU_DIAG=1, logbook 2026-11-20-platform-imu-i2c-error-bursts).  The
 *  error kinds the service can see; the decode table of the repurposed STATUS bytes is in the
 *  sketch's "IMU DIAGNOSTIC BUILD" block.  The kinds exist in every build (no code); everything
 *  that records them is under #if PLATFORM_IMU_DIAG. */
enum DiagKind : uint8_t {
  DK_NONE = 0,
  DK_COUNT = 1,       // FIFO_COUNT read failed (drain)                 -> i2c_errors, recover, count again
  DK_BURST = 2,       // FIFO_DATA burst failed at the Wire level        -> i2c_errors, recover, flush, DISCONTINUITY
  DK_FLUSH = 3,       // FIFO flush failed (overflow/bad packet/burst)   -> i2c_errors, recover, flush again
                      //   (COUNT/BURST/FLUSH: the third failure in a row goes to BACKOFF)
  DK_PROBE = 4,       // an init step failed (PROBING)                  -> i2c_errors, stays PROBING
  DK_RESUME = 5,      // RESUME's flush or WHO_AM_I check failed          -> i2c_errors, PROBING
  DK_BAD_PACKET = 6,  // a FIFO header was not 0b0110_xxxx               -> bad_packets only, flush, DISCONTINUITY
};
#if PLATFORM_IMU_DIAG
/*  What the sketch's Io reports about its LAST operation (diag_op()). */
struct DiagOp {
  uint8_t stage = 0;       // icm42688::Driver::DiagStage
  uint8_t code = 0;        // Wire return / bytes received / bad header byte
  uint8_t requested = 0;   // bytes requested / (burst n << 4 | packet index)
  uint8_t msr_flags = 0;   // LPI2C MSR after the failure: b0 NDF b1 ALF b2 FEF b3 PLTF b4 SDF b5 MBF b6 BBF
  uint32_t dur_us = 0;     // wall time of the op
  uint8_t init_failed_at = 0, int_status = 0, count = 0;
};
inline uint8_t diag_sat8(uint32_t v) { return v > 255u ? uint8_t(255) : uint8_t(v); }
#endif

class Service {
 public:
  enum StepKind : uint8_t { STEP_NONE = 0, STEP_I2C = 1, STEP_CRC = 2, STEP_DATA = 3 };

  /*  The capture must already be init()ed on its storage.  Boots into PROBING (the Engine),
   *  inert until BOOT_SETTLE_MS after `now_ms`, with the first probe then. */
  void init(ImuCapture::Capture* cap, uint32_t boot_id, uint32_t now_ms) {
    cap_ = cap;
    boot_id_ = boot_id;
    eng_.init(cap);
    settle_end_ms_ = now_ms + BOOT_SETTLE_MS;
    settled_ = halted_for_boot_ = false;
    probe_at_ = settle_end_ms_;
    fail_streak_ = retry_streak_ = 0;
    init_started_ = chip_ok_ = resume_who_ = flush_pending_ = recover_pending_ = false;
    new_capture();
  }

  CmdSlots& cmds() { return cmds_; }
  SyncQuality& sync() { return sync_; }
  const ImuProto::Engine& engine() const { return eng_; }
  uint8_t state() const { return eng_.state(); }
  const Counters& counters() const { return cnt_; }
  bool chip_ok() const { return chip_ok_; }
  uint32_t probe_due_ms() const { return probe_at_; }
  uint32_t fail_streak() const { return fail_streak_; }
  uint32_t retry_streak() const { return retry_streak_; }
  StepKind last_step() const { return last_step_; }
  bool settled() const { return settled_; }
  bool halted_for_boot() const { return halted_for_boot_; }

  /*  One loop() pass, after the CAN dispatch and FwUpdate::tick().  `fw_session_open` is
   *  FwUpdate::sessionOpen(): BEGIN opened it, an abandoned or failed session closes it, and
   *  COMMIT never returns (it reboots inside the CAN callback). */
  template <class Io, class Tx>
  void pass(uint32_t now_us, uint32_t now_ms, bool fw_session_open, Io& io, Tx& tx) {
    using namespace PlatformImuState;
    last_step_ = STEP_NONE;
#if PLATFORM_IMU_DIAG
    if (diag_have_prev_ && uint32_t(now_us - diag_prev_us_) > diag_max_gap_us_) diag_max_gap_us_ = now_us - diag_prev_us_;
    diag_prev_us_ = now_us;
    diag_have_prev_ = true;
#endif
    // -1. The boot settle: drop commands, note a BEGIN, nothing else (BOOT_SETTLE_MS).  The
    //     latch is exact while passes come at least every 24.8 days, which loop() guarantees.
    if (!settled_) {
      if (fw_session_open) halted_for_boot_ = true;
      uint8_t sd[8], sl;
      while (cmds_.pop(sd, &sl)) cnt_.cmds_in_settle++;
      if (int32_t(now_ms - settle_end_ms_) < 0) return;
      settled_ = true;
    }
    // 0. Firmware update first: nothing below may touch the bus once BEGIN has been seen.
    if (fw_session_open || halted_for_boot_) {
      if (eng_.state() != FW_HALTED) { eng_.on_fw_update_begin(); halt_local(); }
    } else if (eng_.state() == FW_HALTED) {
      eng_.on_fw_update_abandoned();
      enter_probing(now_ms + PROBE_MIN_GAP_MS);
    }
    // 1. Commands (no I2C: a command only changes state; RESUME's chip check is a later step).
    uint8_t d[8], len;
    while (cmds_.pop(d, &len)) handle_cmd(d, len, now_ms, io);
    // 2. Internal transitions.
    const uint8_t before = eng_.state();
    eng_.tick(now_ms);
    if (before == BACKOFF && eng_.state() == PROBING) init_started_ = false;
    // 3. At most one bounded step.
    step(now_us, now_ms, io);
    // 4. Frames: replies, then status, then (only if no other step ran) one DATA frame.
    send_frames(now_us, now_ms, tx);
  }

 private:
  /* ------------------------------- commands ------------------------------- */
  template <class Io>
  void handle_cmd(const uint8_t* d, uint8_t len, uint32_t now_ms, Io& io) {
    using namespace PlatformImuState;
    const ImuProto::Cmd c = ImuProto::parse_cmd(d, len);
    const uint8_t s0 = eng_.state();
    if (c.valid && c.op == PlatformImuOp::FREEZE && s0 == RUNNING) close_unanchored();  // a last anchor for the tail
    ImuCapture::HeaderInfo info;
    if (c.valid && c.op == PlatformImuOp::FREEZE) build_info(&info, io);
    bool deferred = false;
    const ImuProto::Reply r = eng_.handle(c, info, &deferred);
    if (!deferred) queue_reply(r);
    const uint8_t s1 = eng_.state();
    if (r.status != PlatformImuStatus::OK) return;
    if (c.op == PlatformImuOp::DISABLE && s0 != DISABLED) { halt_local(); return; }
    if (c.op == PlatformImuOp::ENABLE && s0 == DISABLED) { new_capture(); enter_probing(now_ms); return; }
    if (c.op == PlatformImuOp::RESUME && s1 == RUNNING) { new_capture(); phase_ = IDLE; }
  }

  template <class Io>
  void build_info(ImuCapture::HeaderInfo* h, Io& io) {
    io.header_info(h);
    Stamp st;
    io.stamp(&st);
    h->freeze_wall_us = st.wall_us;
    h->freeze_local_us = st.local_us;
    h->temp_start_raw = have_temp_start_ ? temp_data_from_fifo(temp_start_) : 0;
    h->temp_freeze_raw = have_temp_ ? temp_data_from_fifo(temp_last_) : 0;
    h->fifo_overflows = sat16(cap_overflows_);
    h->i2c_errors = sat16(cap_i2c_errors_);
    h->sync_peak_us = sync_.capture_peak_us();
    if (!st.wall_valid) h->extra_flags |= PlatformImuCaptureFlag::NEVER_SYNCED;
  }

  /* -------------------------------- one step -------------------------------- */
  template <class Io>
  void step(uint32_t now_us, uint32_t now_ms, Io& io) {
    using namespace PlatformImuState;
    if (eng_.freeze_pending()) {           // one CRC slice; FROZEN has no I2C
      ImuProto::Reply r;
      if (eng_.service_freeze(&r)) queue_reply(r);
      last_step_ = STEP_CRC;
      return;
    }
    const uint8_t st = eng_.state();
    if (st == PROBING) { probe_step(now_us, now_ms, io); return; }
    if (st != RUNNING) return;             // DISABLED, BACKOFF, FROZEN, UPLOADING, FW_HALTED: no I2C
    if (!eng_.sampling()) {                // a RESUME waits for its FIFO flush
      eng_.take_resume_check();
      if (!chip_ok_) { eng_.on_resume_chip_failed(); enter_probing(now_ms); return; }
      last_step_ = STEP_I2C;
      if (io.flush() < 0) { diag_note(DK_RESUME, io); resume_failed(now_ms); return; }
      resume_who_ = true;
      return;
    }
    if (resume_who_) {
      resume_who_ = false;
      last_step_ = STEP_I2C;
      if (io.check_who() < 0) { diag_note(DK_RESUME, io); resume_failed(now_ms); }
      return;
    }
    if (fail_streak_ && int32_t(now_ms - healthy_since_ms_) >= int32_t(HEALTHY_RESET_MS)) fail_streak_ = 0;
    drain_step(now_us, now_ms, io);
  }

  template <class Io>
  void probe_step(uint32_t now_us, uint32_t now_ms, Io& io) {
    if (int32_t(now_ms - probe_at_) < 0) return;
    /*  UNCONDITIONAL RECOVERY (2026-11-20).  Every counted I2C error leaves the next probe to
     *  start from a cleared bus and a freshly reset I2C controller, as its own step: a device
     *  left mid-transfer (holding SDA) is released by the clocks and reset by the STOP, and a
     *  wedged controller is reset, so nothing short of a dead chip needs a reboot to heal.  The
     *  core's Wire only clocks the bus when BBF stays set for 16 ms and never resets the
     *  controller after begin(); FW 11 was seen stuck in PROBING for a night that a reboot cured. */
    if (!init_started_ && recover_pending_) {
      recover_pending_ = false;
      cnt_.recoveries++;
      last_step_ = STEP_I2C;
      io.bus_recover();
      return;
    }
    if (!init_started_) { io.init_start(); init_started_ = true; cnt_.probes++; }
    last_step_ = STEP_I2C;
    const int r = io.init_step(now_us);
    if (r == INIT_PENDING) return;
    init_started_ = false;
    if (r == 0) {
      chip_ok_ = true;
      healthy_since_ms_ = now_ms;
      phase_ = IDLE;
      flush_pending_ = false;
      retry_streak_ = 0;
      last_drain_us_ = now_us;
      eng_.on_chip_ready();
      return;
    }
    diag_note(DK_PROBE, io);
    count_i2c_error();
    chip_ok_ = false;
    fail(now_ms);
  }

  /*  THE RUNNING RETRY POLICY (FW 12, logbook 2026-11-20-platform-imu-i2c-error-bursts).  The
   *  robot's I2C loses one transfer every 15-40 s (register-address NACK, short read, arbitration
   *  lost); FW 11 sent each one to BACKOFF, a 1 s hole.  Now, while RUNNING:
   *    FIFO_COUNT read fails  nothing was taken from the FIFO: recover the bus on the next pass,
   *                           count again on the pass after (the drain is made due at once).
   *                           No DISCONTINUITY, no sample lost.
   *    burst read fails       the chip's FIFO read pointer is unknown (RESUME_PARTIAL_RD resumes
   *                           mid-packet): drop the rest of the drain, close the anchors, mark the
   *                           next anchor DISCONTINUITY, recover the bus, flush, carry on.
   *    flush fails            recover, flush again (the DISCONTINUITY mark is still pending).
   *  Every failure is counted in i2c_errors.  A successful count, burst or flush resets the
   *  streak; a failure with retry_streak_ == RUNNING_MAX_RETRIES goes to BACKOFF as before.  A
   *  bad FIFO header is not an I2C failure: the transfer completed, so it neither counts nor
   *  resets the streak (it flushes and marks a DISCONTINUITY on its own).  Each step is still one
   *  bounded bus operation per pass, so a failure, a recovery and its retry take three passes. */
  template <class Io>
  void drain_step(uint32_t now_us, uint32_t now_ms, Io& io) {
    if (phase_ == IDLE) {
      if (recover_pending_) {              // after an absorbed error: clear the bus before the next transfer
        recover_pending_ = false;
        cnt_.recoveries++;
        last_step_ = STEP_I2C;
        io.bus_recover();
        return;
      }
      if (flush_pending_) {                // after an overflow, a bad packet or a failed burst: start clean
        last_step_ = STEP_I2C;
        if (io.flush() < 0) { diag_note(DK_FLUSH, io); retry_or_backoff(now_ms); return; }  // stays pending
        flush_pending_ = false;
        retry_streak_ = 0;
        last_drain_us_ = now_us;
        return;
      }
      if (uint32_t(now_us - last_drain_us_) < DRAIN_PERIOD_US) return;
      last_drain_us_ = now_us;
      io.stamp(&cur_);                     // immediately before the FIFO_COUNT read (spec section 3)
      bool ovf = false;
      last_step_ = STEP_I2C;
      const int n = io.fifo_count(&ovf);
      if (n < 0) {
        diag_note(DK_COUNT, io);
        if (retry_or_backoff(now_ms)) last_drain_us_ = now_us - DRAIN_PERIOD_US;  // due again after the recovery
        return;
      }
      retry_streak_ = 0;
      if (ovf) {
#if PLATFORM_IMU_DIAG
        {
          DiagOp d;
          io.diag_op(&d);
          diag_ovf_ = uint8_t((d.count > 127 ? 127 : d.count) | ((d.int_status & 0x02) ? 0x80 : 0));
        }
#endif
        cnt_.fifo_overflows++;
        cap_overflows_++;
        close_unanchored();
        eng_.note_overflow();              // the next anchor is DISCONTINUITY
        flush_pending_ = true;
        return;
      }
      if (n == 0) return;
      remaining_ = uint32_t(n) > DRAIN_MAX_PKTS ? DRAIN_MAX_PKTS : uint32_t(n);
      phase_ = READING;
      return;
    }
    const int n = remaining_ > uint32_t(BURST_PKTS) ? BURST_PKTS : int(remaining_);
    Pkt pk[BURST_PKTS];
    last_step_ = STEP_I2C;
    const int got = io.read_burst(pk, n);
    if (got == ERR_BAD_PACKET) {           // nothing of this burst is pushed; the earlier bursts were sound
      diag_note(DK_BAD_PACKET, io);
      cnt_.bad_packets++;
      drop_drain_and_flush();
      return;
    }
    if (got < 0) {
      diag_note(DK_BURST, io);
      if (retry_or_backoff(now_ms)) drop_drain_and_flush();
      return;
    }
    retry_streak_ = 0;
    for (int i = 0; i < got && i < n; i++) {
      if (!cap_->push(pk[i].s)) continue;
      temp_last_ = pk[i].temp;
      have_temp_ = true;
      if (!have_temp_start_) { temp_start_ = pk[i].temp; have_temp_start_ = true; }
      unanchored_ = true;
      unanchored_stamp_ = cur_;
    }
    remaining_ -= uint32_t(n);
    if (remaining_ == 0) { phase_ = IDLE; complete_drain(now_ms); }
  }

  /*  The FIFO read position is no longer known (a bad header or a failed burst): abandon the
   *  rest of this drain, anchor what was pushed, mark the next anchor DISCONTINUITY and flush
   *  the chip FIFO before the next count.  Stays RUNNING. */
  void drop_drain_and_flush() {
    phase_ = IDLE;
    remaining_ = 0;
    close_unanchored();
    eng_.note_overflow();
    flush_pending_ = true;
  }

  /*  A drain is complete: anchor it if one is due (first of the capture, 100 ms since the last,
   *  or the first complete drain after a discontinuity). */
  void complete_drain(uint32_t now_ms) {
    if (sync_.take_step()) sync_step_pending_ = true;
    const bool disc = eng_.take_discontinuity() || disc_carry_;
    disc_carry_ = false;
    if (!unanchored_) { disc_carry_ = disc; return; }  // nothing pushed: keep the mark for the next one
    const bool due = first_anchor_ || disc || uint32_t(now_ms - last_anchor_ms_) >= PlatformImu::ANCHOR_PERIOD_MS;
    if (!due) return;
    add_anchor(unanchored_stamp_, disc);
    last_anchor_ms_ = now_ms;
  }

  /*  Before a discontinuity (or a freeze): anchor the samples pushed since the last anchor with
   *  the stamp of the drain they came from, so every sample before the gap sits at or before an
   *  ordinary anchor and every sample after it at or before the DISCONTINUITY anchor.  The fit's
   *  segment boundary is then exact (spec section 5). */
  void close_unanchored() {
    if (sync_.take_step()) sync_step_pending_ = true;
    if (unanchored_) add_anchor(unanchored_stamp_, false);
  }

  void add_anchor(const Stamp& s, bool disc) {
    uint8_t f = 0;
    if (disc) f |= PlatformImuAnchorFlag::DISCONTINUITY;
    if (!s.wall_valid) f |= PlatformImuAnchorFlag::WALL_INVALID;
    if (sync_step_pending_) f |= PlatformImuAnchorFlag::SYNC_STEP;
    if (!cap_->add_anchor(s.local_us, s.wall_us, f)) return;
    sync_step_pending_ = false;
    unanchored_ = false;
    first_anchor_ = false;
    if (disc) capture_disc_ = true;
  }

  /*  A failed FIFO transfer while RUNNING: counted, then absorbed (true: recover and retry on
   *  later passes) or, the (RUNNING_MAX_RETRIES + 1)-th in a row, BACKOFF (false). */
  bool retry_or_backoff(uint32_t now_ms) {
    count_i2c_error();                     // also arms the bus recovery
    if (retry_streak_ >= RUNNING_MAX_RETRIES) { running_error(now_ms); return false; }
    retry_streak_++;
    cnt_.retries++;
    return true;
  }
  void running_error(uint32_t now_ms) {
    close_unanchored();
    chip_ok_ = false;
    phase_ = IDLE;
    remaining_ = 0;
    flush_pending_ = false;
    retry_streak_ = 0;
    eng_.on_i2c_error(now_ms);             // RUNNING -> BACKOFF, the next anchor DISCONTINUITY
    fail(now_ms);
  }
  void resume_failed(uint32_t now_ms) {
    count_i2c_error();
    chip_ok_ = false;
    eng_.on_resume_chip_failed();          // RUNNING -> PROBING (spec: "or PROBING if the chip fails")
    init_started_ = false;
    fail(now_ms);
  }
  void fail(uint32_t now_ms) {
    fail_streak_++;
    probe_at_ = now_ms + backoff_ms(fail_streak_);
  }
  void count_i2c_error() { cnt_.i2c_errors++; cap_i2c_errors_++; recover_pending_ = true; }

  /*  Diagnostic build only: record what the Io says about the op that just failed.  An empty
   *  inline call in the normal build. */
  template <class Io>
  void diag_note(uint8_t kind, Io& io) {
#if PLATFORM_IMU_DIAG
    DiagOp d;
    io.diag_op(&d);
    diag_kind_ = kind;
    diag_stage_ = d.stage;
    diag_code_ = d.code;
    diag_req_ = d.requested;
    diag_msr_ = d.msr_flags;
    diag_dur100_ = diag_sat8((d.dur_us + 99u) / 100u);
    if (kind < sizeof diag_n_ / sizeof diag_n_[0]) diag_n_[kind]++;
    if (kind == DK_PROBE) diag_init_at_ = d.init_failed_at;
#else
    (void)kind;
    (void)io;
#endif
  }

  void enter_probing(uint32_t probe_at_ms) {
    init_started_ = false;
    probe_at_ = probe_at_ms;
  }
  /*  FW_HALTED or DISABLED: the capture is gone and no I2C runs until PROBING re-inits the chip. */
  void halt_local() {
    init_started_ = chip_ok_ = resume_who_ = flush_pending_ = false;
    retry_streak_ = 0;
    new_capture();
  }
  void new_capture() {
    phase_ = IDLE;
    remaining_ = 0;
    retry_streak_ = 0;
    unanchored_ = disc_carry_ = sync_step_pending_ = capture_disc_ = false;
    first_anchor_ = true;
    have_temp_start_ = false;
    cap_overflows_ = cap_i2c_errors_ = 0;
    sync_.reset_capture();
  }

  /* --------------------------------- frames --------------------------------- */
  void queue_reply(const ImuProto::Reply& r) {
    const uint8_t next = uint8_t((r_head_ + 1) % REPLY_SLOTS);
    if (next == r_tail_) { cnt_.tx_deferred++; return; }  // full: the host's RPC retry covers it
    replies_[r_head_] = ImuProto::pack_reply(r);
    r_head_ = next;
  }

  template <class Tx>
  void send_frames(uint32_t now_us, uint32_t now_ms, Tx& tx) {
    while (r_tail_ != r_head_) {
      if (!tx.place(PlatformCanId::IMU_REPLY, replies_[r_tail_].d)) { cnt_.tx_deferred++; return; }
      r_tail_ = uint8_t((r_tail_ + 1) % REPLY_SLOTS);
    }
    if (eng_.status_due(now_ms)) { status_ = build_status(); status_pending_ = true; }
    if (status_pending_) {
      if (!tx.place(PlatformCanId::IMU_STATUS, status_.d)) { cnt_.tx_deferred++; return; }
      status_pending_ = false;
    }
    if (last_step_ != STEP_NONE) return;   // one step per pass: a CRC slice or I2C already ran
    ImuProto::Frame f;
    if (!eng_.peek_data(now_us, &f)) return;
    if (!tx.place(PlatformCanId::IMU_DATA, f.d)) { cnt_.tx_deferred++; return; }
    eng_.commit_data(now_us);
    last_step_ = STEP_DATA;
  }

  ImuProto::Frame build_status() {
    ImuProto::Status s;
    s.state = eng_.state();
    uint8_t peak = 255;
    bool synced = false;
    sync_.take_period(&peak, &synced);
    if (chip_ok_) s.flags |= PlatformImuStatusFlag::SENSOR_OK;
    if (synced) s.flags |= PlatformImuStatusFlag::TIME_SYNCED;
    if (cap_->wrapped()) s.flags |= PlatformImuStatusFlag::RING_WRAPPED;
    if (i2c_400k_) s.flags |= PlatformImuStatusFlag::I2C_400K;
    if (capture_disc_) s.flags |= PlatformImuStatusFlag::DISCONTINUITY;
    s.temp_c = have_temp_ ? temp_c_from_fifo(temp_last_) : int8_t(-128);
    s.fifo_overflows = ImuProto::sat_u8(cnt_.fifo_overflows);
    s.i2c_errors = ImuProto::sat_u8(cnt_.i2c_errors);
    s.sync_peak = peak;
    s.fill_s = ImuProto::fill_seconds(cap_->n_samples(), PlatformImu::NOMINAL_ODR_HZ);
    s.boot = uint8_t(boot_id_);
#if PLATFORM_IMU_DIAG
    diag_page(&s, peak);
#endif
    return ImuProto::pack_status(s);
  }

  static uint16_t sat16(uint32_t v) { return v > 0xFFFFu ? 0xFFFFu : uint16_t(v); }

#if PLATFORM_IMU_DIAG
  /*  The diagnostic STATUS: temp_c := page (0..7, one per second), sync_peak := A, fill_s := B.
   *  Decode table: Teensy_code_platform.ino, "IMU DIAGNOSTIC BUILD". */
  void diag_page(ImuProto::Status* s, uint8_t peak) {
    const uint8_t page = uint8_t(diag_seq_++ & 7u);
    uint8_t a = 0, b = 0;
    switch (page) {
      case 0: a = diag_kind_; b = diag_stage_; break;
      case 1: a = diag_code_; b = diag_req_; break;
      case 2: a = diag_msr_; b = diag_dur100_; break;
      case 3: a = diag_sat8(diag_n_[DK_COUNT]); b = diag_sat8(diag_n_[DK_BURST]); break;
      case 4: a = diag_sat8(diag_n_[DK_FLUSH] + diag_n_[DK_RESUME]); b = diag_sat8(diag_n_[DK_PROBE]); break;
      case 5: a = diag_sat8(diag_n_[DK_BAD_PACKET]); b = diag_sat8(cnt_.recoveries); break;
      case 6:
        a = peak;
        b = diag_sat8((diag_max_gap_us_ + 999u) / 1000u);
        diag_max_gap_us_ = 0;
        break;
      default: a = diag_init_at_; b = diag_ovf_; break;
    }
    s->temp_c = int8_t(page);
    s->sync_peak = a;
    s->fill_s = b;
  }
#endif

 public:
  void set_i2c_400k(bool v) { i2c_400k_ = v; }

 private:
  enum Phase : uint8_t { IDLE = 0, READING = 1 };

  ImuCapture::Capture* cap_ = nullptr;
  ImuProto::Engine eng_;
  CmdSlots cmds_;
  SyncQuality sync_;
  Counters cnt_;
  uint32_t boot_id_ = 0;
  bool i2c_400k_ = false;
  StepKind last_step_ = STEP_NONE;
  // boot settle
  uint32_t settle_end_ms_ = 0;
  bool settled_ = false, halted_for_boot_ = false;
  // probe and back-off
  uint32_t probe_at_ = 0, fail_streak_ = 0, healthy_since_ms_ = 0;
  uint32_t retry_streak_ = 0;              // consecutive failed FIFO transfers absorbed in RUNNING
  bool init_started_ = false, chip_ok_ = false, resume_who_ = false, recover_pending_ = false;
  // drain
  Phase phase_ = IDLE;
  uint32_t remaining_ = 0, last_drain_us_ = 0;
  bool flush_pending_ = false;
  Stamp cur_, unanchored_stamp_;
  // anchors and per-capture header figures
  bool unanchored_ = false, first_anchor_ = true, disc_carry_ = false, sync_step_pending_ = false, capture_disc_ = false;
  uint32_t last_anchor_ms_ = 0, cap_overflows_ = 0, cap_i2c_errors_ = 0;
  int8_t temp_last_ = 0, temp_start_ = 0;
  bool have_temp_ = false, have_temp_start_ = false;
  // frames
  ImuProto::Frame replies_[REPLY_SLOTS] = {};
  uint8_t r_head_ = 0, r_tail_ = 0;
  ImuProto::Frame status_ = {};
  bool status_pending_ = false;
#if PLATFORM_IMU_DIAG
  uint8_t diag_kind_ = 0, diag_stage_ = 0, diag_code_ = 0, diag_req_ = 0, diag_msr_ = 0, diag_dur100_ = 0;
  uint8_t diag_init_at_ = 0, diag_ovf_ = 0;
  uint32_t diag_n_[8] = {};
  uint32_t diag_seq_ = 0, diag_prev_us_ = 0, diag_max_gap_us_ = 0;
  bool diag_have_prev_ = false;

 public:
  uint8_t diag_kind() const { return diag_kind_; }
#endif
};

}  // namespace ImuService
