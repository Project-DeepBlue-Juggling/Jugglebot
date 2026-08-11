// =============================================================================
//  time_base.cpp — monotonic + wall-clock implementation
// =============================================================================
#include "time_base.h"

#include <Arduino.h>
#include "udp_protocol.h"   // JbUdp::ClockDiagFlags — the flag bits are wire-defined,
                            // so the generator owns them and this file never re-declares them

namespace CanBridge {

uint64_t micros64() {
  // Extend 32-bit micros() to 64 bits by counting wraps. Unlike the platform
  // Teensy (single-threaded loop), this is called from BOTH the 500 Hz ISR and
  // FreeRTOS tasks, so the read-modify-write of the wrap statics must be atomic —
  // guard the whole body (PRIMASK save/restore → safe from ISR context too).
  static uint32_t last_lo = micros();
  static uint64_t hi = 0;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const uint32_t now = micros();
  if (now < last_lo) hi += (uint64_t)1 << 32;
  last_lo = now;
  const uint64_t v = hi | now;
  __set_PRIMASK(pm);
  return v;
}

static volatile int64_t  s_wall_offset_us  = 0;   // now_wall_us = micros64() + offset
static volatile bool     s_have_offset     = false;
static volatile uint64_t s_last_anchor_us  = 0;   // micros64() at the last accepted anchor (freshness)

uint64_t now_wall_us() {
  // s_wall_offset_us is 64-bit and slewed by the time-sync task while the ISR
  // reads it here — read it atomically to avoid a torn value.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const int64_t off = s_wall_offset_us;
  __set_PRIMASK(pm);
  return micros64() + (uint64_t)off;
}

bool time_synced() {
  // Synced only if anchored AND the anchor is fresh — a stale anchor (Jetson
  // link down) means the clock is free-running/drifting and must not be trusted
  // (this self-gates the 0x7DD broadcast so slaves go unsynced instead of
  // following a drifting master).
  if (!s_have_offset) return false;
  const uint64_t last = atomic_read_u64(&s_last_anchor_us);
  return (micros64() - last) < TIME_ANCHOR_STALE_US;
}

// ── Per-anchor diagnostic state (FW 11) ──────────────────────────────────────
// Written ONLY by set_wall_anchor, which has exactly ONE caller
// (time_sync_master.cpp::on_tod_response, net-task context) — verified by grep,
// and that single-writer property is what the split critical sections below rest
// on. Read by task_time_sync via clock_last_anchor().
static ClockAnchorSample s_anchor{};
static volatile bool s_have_anchor   = false;
// Previous MEASUREMENT (not the servo's slewed state), for the implied-frequency
// estimate. Differencing measurements makes freq_ppb open-loop with respect to
// the IIR: it reports the crystal, not the filter's response to the crystal.
static bool     s_have_prev_meas   = false;
static int64_t  s_prev_meas_offset = 0;
static uint64_t s_prev_meas_local  = 0;
static uint32_t s_anchor_seq       = 0;

void set_wall_anchor(uint64_t jetson_wall_us) {
  const int64_t local = (int64_t)micros64();
  const int64_t offset = (int64_t)jetson_wall_us - local;
  int64_t diff = 0;
  bool first = false, stepped = false;
  // Atomic read-modify-write of the 64-bit offset (the ISR reads it via now_wall_us).
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  if (!s_have_offset) {
    s_wall_offset_us = offset;    // first anchor → step
    s_have_offset = true;
    first = true;
    stepped = true;               // the first anchor IS a step (of the whole epoch)
  } else {
    diff = offset - s_wall_offset_us;
    if (diff > TIME_STEP_THRESHOLD_US || diff < -TIME_STEP_THRESHOLD_US) {
      // Large error → boot / re-acquisition after a sync gap. Step: slewing this
      // at 1/16-every-30 s would take ~30 min and reads as a thrower warm-up drift.
      s_wall_offset_us = offset;
      stepped = true;
    } else {
      // Small drift → slew via the IIR (smooth, monotonic).
      s_wall_offset_us += diff >> TIME_OFFSET_IIR_SHIFT;
    }
  }
  s_last_anchor_us = (uint64_t)local;
  __set_PRIMASK(pm);
  // Everything the servo needs is done. The rest is diagnostics, and it runs
  // OUTSIDE the critical section ON PURPOSE: the block below contains a 64-bit
  // divide (a libgcc software routine on this core), and the 500 Hz interp ISR
  // must never eat that as jitter. Only the ~30-byte publish at the end re-masks.
  // Capturing `diff`/`stepped`/`first` into locals above is the only cost added
  // inside the IRQ-off window — three register stores.

  // ── err_us: the pre-slew phase error, saturated ──
  // On the slew branch |diff| <= TIME_STEP_THRESHOLD_US (20 ms) by construction,
  // so i32 is exact there — the branch that matters for the servo work. A STEP
  // can carry an arbitrary jump (a Jetson NTP correction), so saturate rather
  // than wrap: a saturated +-2147 s reads as "off the scale", a wrapped one reads
  // as a plausible small number with the wrong sign.
  int32_t err32 = 0;
  if (!first) {
    if (diff > (int64_t)INT32_MAX)      err32 = INT32_MAX;
    else if (diff < (int64_t)INT32_MIN) err32 = INT32_MIN;
    else                                err32 = (int32_t)diff;
  }

  // ── freq_ppb: implied fractional frequency error over the last interval ──
  uint32_t dt32 = 0;
  int32_t  freq_ppb = 0;
  bool     freq_valid = false;
  if (s_have_prev_meas) {
    const uint64_t dt64 = (uint64_t)local - s_prev_meas_local;   // micros64: monotonic, wrap-safe
    dt32 = (dt64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFu : (uint32_t)dt64;
    const int64_t dmeas = offset - s_prev_meas_offset;
    // Four independent reasons a sample is not a crystal measurement, all of
    // which must clear FREQ_VALID rather than produce a plausible-looking number:
    //   * this anchor STEPPED  → the interval contains a non-crystal event
    //     (boot, a link gap, a Jetson clock step); >20 ms in 30 s is >666 ppm,
    //     which no crystal does;
    //   * dt64 == 0            → division by zero;
    //   * dt64 saturated       → the reported denominator is not the real one;
    //   * |dmeas| > 1e9 us     → keeps dmeas * 1e9 inside int64 by construction
    //     (1e9 * 1e9 = 1e18 < 9.22e18) instead of by assumption.
    if (!stepped && dt64 > 0 && dt64 <= 0xFFFFFFFFULL &&
        dmeas <= 1000000000LL && dmeas >= -1000000000LL) {
      int64_t ppb = (dmeas * 1000000000LL) / (int64_t)dt64;
      if (ppb > (int64_t)INT32_MAX)      ppb = INT32_MAX;
      else if (ppb < (int64_t)INT32_MIN) ppb = INT32_MIN;
      freq_ppb = (int32_t)ppb;
      freq_valid = true;
    }
  }
  s_prev_meas_offset = offset;
  s_prev_meas_local  = (uint64_t)local;
  s_have_prev_meas   = true;
  ++s_anchor_seq;

  uint8_t flags = 0;
  if (stepped)    flags |= JbUdp::ClockDiagFlags::STEPPED;
  if (first)      flags |= JbUdp::ClockDiagFlags::FIRST_ANCHOR;
  if (freq_valid) flags |= JbUdp::ClockDiagFlags::FREQ_VALID;

  // Publish the sample as one atomic step. Same reasoning as
  // interp_on_setpoint's staging publish: a multi-word struct written
  // field-by-field can be read as a mix of two anchors by a preempting reader,
  // and a mixed clock sample is a FABRICATED data point, not noise. ~30 bytes
  // under PRIMASK is well under a microsecond, against a 2 ms interp period, once
  // per 30 s.
  const uint32_t pm2 = __get_PRIMASK(); __disable_irq();
  s_anchor.t_local_us     = (uint64_t)local;
  s_anchor.jetson_wall_us = jetson_wall_us;
  s_anchor.dt_local_us    = dt32;
  s_anchor.err_us         = err32;
  s_anchor.freq_ppb       = freq_ppb;
  s_anchor.anchor_seq     = s_anchor_seq;
  s_anchor.flags          = flags;
  s_have_anchor = true;
  __set_PRIMASK(pm2);
}

bool clock_last_anchor(ClockAnchorSample* out) {
  if (!out) return false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool have = s_have_anchor;
  if (have) *out = s_anchor;
  __set_PRIMASK(pm);
  return have;
}

}  // namespace CanBridge
