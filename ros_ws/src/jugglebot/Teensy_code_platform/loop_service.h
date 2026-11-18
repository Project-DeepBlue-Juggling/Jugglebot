/*****************************************************************************************
 *  loop_service.h — what the Platform Teensy's loop() does each pass (FW 11, IDEA-20 P2a).
 *
 *  Pure logic with no Arduino dependency, so tests/firmware/native/ runs the shipped code
 *  (test_platform_loop_service.cpp). The sketch owns the clock, the SPI and the CAN writes,
 *  and hands them in.
 *
 *  WHY THIS EXISTS.  This board's USB port is dead: the in-sketch firmware-over-CAN
 *  receiver is its only way back in, and that receiver only runs while loop() keeps
 *  calling can1.events().  Before FW 11 three things could stop that:
 *    1. setup() waited FOR EVER for the SCL3300 inclinometer;
 *    2. a tilt request ran the inclinometer inside the CAN callback, and a failing
 *       sensor held it there for at least 600 ms (3 x a 103 ms re-initialisation plus
 *       delay(100)), with no CAN frame dispatched meanwhile;
 *    3. loop() dispatched one received frame per pass (FW 11 keeps that during a
 *       firmware update and drains up to 32 otherwise, see Can below).
 *  FW 11 bounds all three, and this header holds the decisions.
 *
 *  THE INCLINOMETER'S COST, from the SCL3300 library (davidarmstrong/SCL3300 3.3.x):
 *    available() — one read: 12 SPI words with a >= 10 us gap each, then a 1 ms busy
 *                  wait in endTransmission(): about 1.3 ms.
 *    begin()     — a (re)initialisation: 1 + 1 + 100 ms of busy waits plus the 1 ms in
 *                  endTransmission(): about 103 ms, ONE call that cannot be split (the
 *                  word transfer it is built from is private).  reset() is begin().
 *  So a read is the only per-pass operation, and a probe (begin()) is rationed: never in
 *  a CAN callback, never while a firmware update session is open, never in the same pass
 *  as a read, at most one per PROBE_MIN_GAP_MS, backing off to one per PROBE_MAX_GAP_MS,
 *  and never brought forward by a tilt request.
 *
 *  TIME.  Every due time is set as `now + offset` with offset <= PROBE_MAX_GAP_MS, and
 *  compared with reached(), which is exact across the uint32 wrap of millis() (49.7
 *  days) as long as the two times are within 2^31 ms (24.8 days) of each other.
 *****************************************************************************************/
#pragma once

#include <stdint.h>
#include <string.h>

#include "protocol_config.h"  // PlatformTilt::INVALID_NAN_BITS, PlatformTiltReason (generated)

namespace LoopService {

/*  True once `now` has reached `at`.  Wrap-safe (see TIME above). */
inline bool reached(uint32_t now, uint32_t at) { return int32_t(now - at) >= 0; }

/*  The earlier of two due times, wrap-safe on the same terms. */
inline uint32_t earlier(uint32_t a, uint32_t b) { return int32_t(a - b) <= 0 ? a : b; }

/*----------------------------------------------------------------------------*/
/*  CAN dispatch: how many received frames one loop() pass hands to canSniff  */
/*----------------------------------------------------------------------------*/
/*  FlexCAN_T4's events() dispatches ONE queued frame per call and returns
 *  (frames still queued << 12) | (TX frames still queued).  loop() calls it while
 *  may_dispatch() says so, so a pass always ends and the rest of the pass (tilt, the
 *  firmware-update idle timeout, the IMU, the Hand fan) always runs.
 *
 *  32: the bus carries up to about 6800 frames/s (62-75 % load, 111 bits per frame),
 *  6.8 per ms.  A pass that dispatches 32 ordinary frames takes about 32 x 3-5 us, so
 *  the queue drains at better than 6000 frames per ms of dispatch while the work after
 *  the dispatch (a 1.3 ms tilt read at most) lets about 9 frames arrive: one pass drains
 *  more than a read-bearing pass adds.  The 256-frame queue empties in 8 full passes. */
constexpr uint32_t CAN_FRAMES_PER_PASS = 32;

namespace Can {

/*  THE SOFTWARE TX QUEUE (P2 audit B2).  FlexCAN_T4's write() puts a frame in its software
 *  queue when no TX mailbox is free, and events() then writes the queued frame into EVERY
 *  free mailbox and pops the queue once per mailbox (FlexCAN_T4.tpp events(), the
 *  frame.mb == -1 loop): duplicates, and a pop of an empty Circular_Buffer that leaves the
 *  queue jammed.  The TX-complete ISR's own refill is one-for-one and is not affected.
 *  So outside a firmware update this sketch never calls events() while the queue holds a
 *  frame, and never dispatches a frame unless every write it and the rest of the pass can
 *  make still finds a free mailbox:
 *    a dispatched frame's handler writes at most WRITES_PER_FRAME_MAX frames (the Hand fan
 *      command: updateHandFan()'s change status, then sendHandFanStatus(); a state request,
 *      a firmware-update command: one; tilt, time sync, IMU_CMD: none);
 *    after the dispatch the pass writes at most WRITES_AFTER_DISPATCH_MAX more (the tilt
 *      reply, the traffic report, the Hand fan status); the IMU never takes the last
 *      ImuService::TX_RESERVE_MAILBOXES mailboxes.
 *  During a firmware update (BEGIN seen, until the session ends) loop() dispatches exactly
 *  one frame per pass, unconditionally, as FW 10 did: the update path is the one already
 *  proven on this board, so it keeps FW 10's behaviour, software queue included. */
constexpr uint32_t CODE_TX_INACTIVE = 8;          // FLEXCAN_MB_CODE_TX_INACTIVE (the sketch static_asserts it)
constexpr uint32_t WRITES_PER_FRAME_MAX = 2;
constexpr uint32_t WRITES_AFTER_DISPATCH_MAX = 3;
constexpr uint32_t DISPATCH_MIN_FREE_TX = WRITES_PER_FRAME_MAX + WRITES_AFTER_DISPATCH_MAX;  // 5 of the 8

/*  The first TX mailbox: FlexCAN_T4's private mailboxOffset(), restated.  With the RX FIFO
 *  enabled, MB 0-5 are the FIFO and 2 x (RFFN + 1) more hold its filter table; past MAXMB
 *  there is no TX mailbox at all. */
inline uint32_t mailbox_offset(uint32_t maxmb, bool fen, uint32_t rffn) {
  if (!fen) return 0;
  const uint32_t fifo = 6u + ((rffn & 0xFu) + 1u) * 2u;
  return fifo > maxmb ? maxmb : fifo;
}

/*  TX mailboxes whose CODE is TX_INACTIVE (free; the hardware also sets it after a
 *  transmission).  codes[i] is mailbox i's CODE field; only i >= the offset is read, so the
 *  caller never has to touch the FIFO's registers (reading an RX C/S word locks it). */
inline uint32_t free_tx_mailboxes(uint32_t maxmb, bool fen, uint32_t rffn, const uint8_t *codes) {
  uint32_t n = 0;
  for (uint32_t i = mailbox_offset(maxmb, fen, rffn); i < maxmb; ++i)
    if (codes[i] == CODE_TX_INACTIVE) ++n;
  return n;
}

/*  Call events() (again) this pass?  calls_made = events() calls so far this pass,
 *  last_ret = what the last one returned, free_tx / tx_queued = the TX mailboxes free now
 *  and FlexCAN's software TX queue depth now. */
inline bool may_dispatch(uint32_t calls_made, uint64_t last_ret, bool fw_session_open,
                         uint32_t free_tx, uint32_t tx_queued) {
  if (fw_session_open) return calls_made == 0;            // FW 10: one events() per pass
  if (calls_made >= CAN_FRAMES_PER_PASS) return false;
  if (calls_made > 0 && (last_ret >> 12) == 0) return false;  // nothing left to dispatch
  return tx_queued == 0 && free_tx >= DISPATCH_MIN_FREE_TX;
}

}  // namespace Can

/*----------------------------------------------------------------------------*/
/*  Tilt: the 0x7DE request/reply, served from loop(), never from the callback  */
/*----------------------------------------------------------------------------*/
namespace Tilt {

constexpr uint8_t  READ_ATTEMPTS       = 3;      // reads per request before an invalid reply
constexpr uint32_t READ_RETRY_GAP_MS   = 10;     // between those reads, across loop passes
constexpr uint8_t  BOOT_PROBE_ATTEMPTS = 3;      // begin() calls in setup(): <= ~310 ms
constexpr uint32_t PROBE_MIN_GAP_MS    = 1000;   // start-to-start floor between two probes
constexpr uint32_t PROBE_MAX_GAP_MS    = 60000;  // the back-off ceiling while the sensor stays down

/*  What the sketch sends on 0x7DE.  NOT_READY and READ_FAILED are the explicit
 *  invalid reply (owner, 2026-11-18): never silence, never a stale value. */
enum class Reply : uint8_t { VALID = 0, NOT_READY = 1, READ_FAILED = 2 };

/*  The 8 reply bytes.  VALID: f32 LE tiltX @0, f32 LE tiltY @4, radians (unchanged
 *  since FW 1).  Invalid: u32 LE INVALID_NAN_BITS @0 and INVALID_NAN_BITS | reason @4,
 *  both quiet float32 NaNs, so a host that decodes '<ff' reads (nan, nan) and its
 *  |tilt| bound rejects it (config/protocol_config.yaml platform_tilt). */
inline void encode_valid(float tilt_x, float tilt_y, uint8_t out[8]) {
  memcpy(out, &tilt_x, 4);
  memcpy(out + 4, &tilt_y, 4);
}

inline void put_u32_le(uint32_t v, uint8_t *p) {
  p[0] = uint8_t(v);
  p[1] = uint8_t(v >> 8);
  p[2] = uint8_t(v >> 16);
  p[3] = uint8_t(v >> 24);
}

inline void encode_invalid(Reply why, uint8_t out[8]) {
  const uint8_t reason = (why == Reply::READ_FAILED) ? PlatformTiltReason::READ_FAILED
                                                     : PlatformTiltReason::NOT_READY;
  put_u32_le(PlatformTilt::INVALID_NAN_BITS, out);
  put_u32_le(PlatformTilt::INVALID_NAN_BITS | reason, out + 4);
}

/*  The service.  `Io` is anything with
 *      bool read();          one inclinometer.available() and, on success, keep the angles
 *      bool probe();         one inclinometer.begin()
 *      void reply(Reply r);  send the 0x7DE reply (VALID uses the angles read() kept)
 *  service() makes AT MOST ONE read() or probe() call per loop pass. */
class Service {
 public:
  /*  setup() probed the sensor (bounded, BOOT_PROBE_ATTEMPTS); this is the verdict. */
  void boot(bool sensor_ok, uint32_t now) {
    ok_ = sensor_ok;
    pending_ = false;
    attempts_ = 0;
    retry_at_ = now;
    last_probe_ = now;
    probe_gap_ = PROBE_MIN_GAP_MS;
    probe_at_ = now + PROBE_MIN_GAP_MS;
  }

  /*  One loop pass.  `request` = at least one 0x7DE request arrived since the last pass. */
  template <class Io>
  void service(uint32_t now, bool request, bool fw_session_open, Io &io) {
    if (request) {
      if (pending_) {
        /* A request during a retry sequence: the reply that sequence ends in answers
         * both.  Restarting it would let a stream of requests postpone every reply. */
      } else if (!ok_) {
        /* No probe pulled forward (P2 audit B1): a 103 ms probe per request would let a
         * host polling a dead sensor take 10 % of every second off the CAN dispatch.  The
         * re-probe schedule depends only on the sensor, never on how often it is asked. */
        io.reply(Reply::NOT_READY);
      } else {
        pending_ = true;
        attempts_ = 0;
        retry_at_ = now;  // the first read is this pass: healthy latency is unchanged
      }
    }

    if (pending_) {
      if (!reached(now, retry_at_)) return;
      if (io.read()) {
        pending_ = false;
        io.reply(Reply::VALID);
        return;
      }
      if (++attempts_ < READ_ATTEMPTS) {
        retry_at_ = now + READ_RETRY_GAP_MS;
        return;
      }
      pending_ = false;
      ok_ = false;
      io.reply(Reply::READ_FAILED);
      /* Re-initialise on a later pass (never this one: one sensor operation per pass),
       * as soon as the floor allows; a glitch is usually cleared before the host's retry. */
      probe_gap_ = PROBE_MIN_GAP_MS;
      probe_at_ = now + gap_left(now);
      return;
    }

    if (ok_ || fw_session_open || !reached(now, probe_at_)) return;
    last_probe_ = now;
    if (io.probe()) {
      ok_ = true;
      probe_gap_ = PROBE_MIN_GAP_MS;
      return;
    }
    probe_gap_ = (probe_gap_ >= PROBE_MAX_GAP_MS / 2) ? PROBE_MAX_GAP_MS : probe_gap_ * 2;
    probe_at_ = now + probe_gap_;
  }

  bool sensor_ok() const { return ok_; }
  bool request_pending() const { return pending_; }
  uint32_t probe_due() const { return probe_at_; }

 private:
  /*  Time left before the start-to-start probe floor is met (0 if it already is).  An
   *  elapsed time aliased by a 49.7-day wrap only delays a probe by < PROBE_MIN_GAP_MS. */
  uint32_t gap_left(uint32_t now) const {
    const uint32_t since = now - last_probe_;
    return since >= PROBE_MIN_GAP_MS ? 0u : PROBE_MIN_GAP_MS - since;
  }

  bool ok_ = false;
  bool pending_ = false;
  uint8_t attempts_ = 0;
  uint32_t retry_at_ = 0;
  uint32_t last_probe_ = 0;
  uint32_t probe_at_ = 0;
  uint32_t probe_gap_ = PROBE_MIN_GAP_MS;
};

}  // namespace Tilt
}  // namespace LoopService
