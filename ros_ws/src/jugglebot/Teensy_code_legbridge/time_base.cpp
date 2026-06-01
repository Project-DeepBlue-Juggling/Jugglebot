// =============================================================================
//  time_base.cpp — monotonic + wall-clock implementation
// =============================================================================
#include "time_base.h"

#include <Arduino.h>

namespace LegBridge {

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

static volatile int64_t s_wall_offset_us = 0;   // now_wall_us = micros64() + offset
static volatile bool    s_have_offset    = false;

uint64_t now_wall_us() {
  // s_wall_offset_us is 64-bit and slewed by the time-sync task while the ISR
  // reads it here — read it atomically to avoid a torn value.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const int64_t off = s_wall_offset_us;
  __set_PRIMASK(pm);
  return micros64() + (uint64_t)off;
}

bool time_synced() {
  return s_have_offset;
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
    // Slew toward the new offset so the broadcast clock stays monotonic and the
    // slave IIR filters never see a backward step.
    const int64_t diff = offset - s_wall_offset_us;
    s_wall_offset_us += diff >> TIME_OFFSET_IIR_SHIFT;
  }
  __set_PRIMASK(pm);
}

}  // namespace LegBridge
