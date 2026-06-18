// =============================================================================
//  time_base.cpp — monotonic + wall-clock implementation
// =============================================================================
#include "time_base.h"

#include <Arduino.h>

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

void set_wall_anchor(uint64_t jetson_wall_us) {
  const int64_t local = (int64_t)micros64();
  const int64_t offset = (int64_t)jetson_wall_us - local;
  // Atomic read-modify-write of the 64-bit offset (the ISR reads it via now_wall_us).
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  if (!s_have_offset) {
    s_wall_offset_us = offset;    // first anchor → step
    s_have_offset = true;
  } else {
    const int64_t diff = offset - s_wall_offset_us;
    if (diff > TIME_STEP_THRESHOLD_US || diff < -TIME_STEP_THRESHOLD_US) {
      // Large error → boot / re-acquisition after a sync gap. Step: slewing this
      // at 1/16-every-30 s would take ~30 min and reads as a thrower warm-up drift.
      s_wall_offset_us = offset;
    } else {
      // Small drift → slew via the IIR (smooth, monotonic).
      s_wall_offset_us += diff >> TIME_OFFSET_IIR_SHIFT;
    }
  }
  s_last_anchor_us = (uint64_t)local;
  __set_PRIMASK(pm);
}

}  // namespace CanBridge
