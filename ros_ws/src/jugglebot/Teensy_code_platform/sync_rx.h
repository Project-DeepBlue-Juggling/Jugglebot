#pragma once
/*============================================================================
 *  sync_rx.h — the Platform Teensy's 0x7DD clock-sync slave (FW 15, BUG-132)
 *============================================================================
 *  Pure C++ (stdint only), so tests/firmware/native/test_platform_sync_rx.cpp
 *  compiles it on the host.  The sketch owns the ISR hook, the IRQ masking and
 *  the globals; everything that decides a number lives here.
 *
 *  THE FAILURE THIS PREVENTS.  Up to FW 14 the beacon was paired with
 *  micros64() when loop() DISPATCHED it, after FlexCAN_T4's 256-slot software
 *  RX ring.  The framework library's `_available` leak turned that ring into a
 *  standing delay line (up to 255 frames, ~114 ms on an idle bus, ~45 ms Armed),
 *  so the Platform wall clock ran early by the queue delay and moved by tens of
 *  ms whenever the bus rate changed (the 572D jump, +61 ms at a re-arm).  FW 15
 *  removes the leak (patched library) AND makes the clock immune to any queue:
 *
 *   1. StampRing: the beacon is stamped in the CAN RX ISR (FlexCAN_T4's
 *      `ext_output1` hook, called before the frame enters the ring), so the sample is (bridge wall time,
 *      local time of RECEPTION).  A queue, a 103 ms tilt re-probe stall or a
 *      dispatch gate after reception cannot shift it.
 *   2. Envelope: every remaining error in a sample is an added delay (the bridge
 *      stamps before its TX queue, the 0x7DD id loses arbitration to everything,
 *      ISR latency), and a delay can only make (wall - local) SMALLER.  So the
 *      best estimate is the LARGEST sample in a recent window, not a mean.
 *
 *  Why not the FlexCAN hardware RX timestamp: it is 16 bits of bit-time (1 us
 *  at 1 Mbit/s) and wraps every 65.5 ms, so extending it to micros64() needs a
 *  pairing made in the ISR anyway; the ISR stamp alone is within microseconds
 *  of it (the ISR fires when the frame lands in the 6-deep hardware FIFO) and
 *  has no wrap to get wrong.
 *==========================================================================*/
#include <stdint.h>

namespace SyncRx {

struct Stamp {
  uint64_t wall_us;      // the beacon's payload: bridge wall time, us
  uint64_t rx_local_us;  // micros64() in the CAN RX ISR
};

/*  Written by the CAN RX ISR (one writer, never re-entered); read from loop() with the
 *  CAN interrupt MASKED by the caller, so a read can never see a half-written slot.
 *  Overwrite-oldest: the ISR never waits.  16 slots = 160 ms of beacons at 100 Hz,
 *  more than the longest loop() stall (the ~103 ms inclinometer re-probe). */
class StampRing {
 public:
  static constexpr uint32_t SLOTS = 16;
  void push(uint64_t wall_us, uint64_t rx_local_us) {   // ISR context
    Stamp& s = slot_[written_ % SLOTS];
    s.wall_us = wall_us;
    s.rx_local_us = rx_local_us;
    written_ = written_ + 1;
  }
  uint32_t written() const { return written_; }
  /*  Entry n (n counts pushes since boot, wrap-safe): false if not yet written or overwritten. */
  bool get(uint32_t n, Stamp* out) const {
    const uint32_t age = written_ - n;
    if (age == 0 || age > SLOTS) return false;
    *out = slot_[n % SLOTS];
    return true;
  }
  /*  The newest retained entry whose payload is wall_us (the dispatch-delay match). */
  bool find(uint64_t wall_us, uint64_t* rx_local_us) const {
    const uint32_t w = written_;
    for (uint32_t k = 1; k <= SLOTS && k <= w; ++k) {
      const Stamp& s = slot_[(w - k) % SLOTS];
      if (s.wall_us == wall_us) { *rx_local_us = s.rx_local_us; return true; }
    }
    return false;
  }

 private:
  Stamp slot_[SLOTS] = {};
  volatile uint32_t written_ = 0;
};

/*  The loop()-side cursor over a StampRing: each entry once, in order; entries the ISR
 *  overwrote before loop() got to them are skipped and counted. */
class Reader {
 public:
  bool take(const StampRing& r, Stamp* out) {
    const uint32_t w = r.written();
    if (w - next_ > StampRing::SLOTS) { lost_ += (w - next_) - StampRing::SLOTS; next_ = w - StampRing::SLOTS; }
    if (next_ == w) return false;
    if (!r.get(next_, out)) return false;
    ++next_;
    return true;
  }
  uint32_t lost() const { return lost_; }

 private:
  uint32_t next_ = 0, lost_ = 0;
};

/*  The offset estimator: offset = wall - local, estimated as the maximum sample over a
 *  sliding window of BLOCKS blocks of BLOCK_US local time (the current partial block plus
 *  the previous BLOCKS - 1 whole ones: between 0.9 and 1.0 s of history).
 *
 *  Behaviour, all deterministic and integer:
 *  - First sample, or the first after a silence longer than the window (bridge reboot,
 *    unplugged bus): the estimate STEPS to it.
 *  - A sample above the estimate (a forward bridge clock step, or upward drift) is taken
 *    at once: the estimate rises on that beacon.
 *  - Samples below it change nothing until the window has no higher sample left.  So a
 *    run of delayed beacons shorter than 0.9 s is ignored entirely, and a genuine
 *    BACKWARD step (the bridge steps its clock when its error exceeds 20 ms) is followed
 *    within 1.0 s plus one beacon.  The two cannot be told apart from the samples alone:
 *    a delay present in EVERY beacon for more than 1.0 s is adopted in full, exactly as a
 *    backward step must be.  That is why the ISR stamp (no queue in the sample) carries
 *    the weight, and the envelope only removes what is left (bridge TX arbitration,
 *    0 to ~2 ms, picked at its minimum over ~100 beacons instead of averaged in).
 *  - Drift (~25 us/s between the two crystals): rising, the estimate follows each beacon;
 *    falling, it holds the window's oldest maximum, a bias of at most 25 us/s x 1.0 s =
 *    25 us. */
class Envelope {
 public:
  static constexpr uint64_t BLOCK_US = 100000;
  static constexpr uint32_t BLOCKS = 10;
  // An estimate move above this reports as a step (SYNC_STEP: the Jetson splits its wall fit there).  Above the
  // bridge's 0-2 ms TX jitter, so the window's maximum ageing out never reads as a step; far below the 20 ms at
  // which the bridge steps its own clock.
  static constexpr int64_t STEP_REPORT_US = 5000;

  struct Result {
    bool first;              // no estimate before this sample (boot, or after a long silence)
    bool stepped;            // first, or the estimate moved by more than STEP_REPORT_US
    int64_t innovation_us;   // sample minus the estimate before this update (0 when first)
  };

  Result update(uint64_t local_us, int64_t sample_us) {
    Result r{false, false, 0};
    const uint64_t blk = local_us / BLOCK_US;
    if (locked_ && blk > cur_ && blk - cur_ >= BLOCKS) locked_ = false;  // the whole window aged out
    if (!locked_) {
      for (uint32_t i = 0; i < BLOCKS; ++i) valid_[i] = false;
      cur_ = blk;
      max_[cur_ % BLOCKS] = sample_us;
      valid_[cur_ % BLOCKS] = true;
      est_ = sample_us;
      locked_ = true;
      r.first = r.stepped = true;
      return r;
    }
    r.innovation_us = sample_us - est_;
    if (blk > cur_) {  // a monotonic clock never goes back; an equal or earlier block is the current one
      for (uint64_t b = cur_ + 1; b <= blk; ++b) valid_[b % BLOCKS] = false;
      cur_ = blk;
    }
    const uint32_t i = uint32_t(cur_ % BLOCKS);
    if (!valid_[i] || sample_us > max_[i]) { max_[i] = sample_us; valid_[i] = true; }
    int64_t m = max_[i];
    for (uint32_t k = 0; k < BLOCKS; ++k)
      if (valid_[k] && max_[k] > m) m = max_[k];
    const int64_t d = m - est_;
    est_ = m;
    r.stepped = d > STEP_REPORT_US || d < -STEP_REPORT_US;
    return r;
  }
  bool locked() const { return locked_; }
  int64_t estimate() const { return est_; }

 private:
  int64_t max_[BLOCKS] = {};
  bool valid_[BLOCKS] = {};
  uint64_t cur_ = 0;
  int64_t est_ = 0;
  bool locked_ = false;
};

/*  The beacon's 8-byte payload: [sec u32 LE][usec u32 LE] of bridge wall time. */
inline uint64_t decode_wall_us(const uint8_t* b) {
  const uint32_t sec = uint32_t(b[0]) | (uint32_t(b[1]) << 8) | (uint32_t(b[2]) << 16) | (uint32_t(b[3]) << 24);
  const uint32_t usec = uint32_t(b[4]) | (uint32_t(b[5]) << 8) | (uint32_t(b[6]) << 16) | (uint32_t(b[7]) << 24);
  return uint64_t(sec) * 1000000ull + usec;
}

/*  A reception-to-dispatch delay in the export's 10 us counts: rounded up, 0xFFFE
 *  saturating.  UNMATCHED (the stamp left the 16-slot ring: over ~150 ms) also reads
 *  0xFFFE, a lower bound.  0xFFFF is reserved for "no beacon dispatched". */
constexpr uint32_t DELAY_UNMATCHED = 0xFFFFFFFFu;
inline uint16_t delay_10us(uint32_t delay_us) {
  if (delay_us == DELAY_UNMATCHED) return 0xFFFE;
  const uint32_t c = delay_us / 10 + (delay_us % 10 ? 1u : 0u);
  return c > 0xFFFE ? uint16_t(0xFFFE) : uint16_t(c);
}

}  // namespace SyncRx
