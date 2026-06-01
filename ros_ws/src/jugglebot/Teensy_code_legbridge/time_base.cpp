// =============================================================================
//  time_base.cpp — monotonic + wall-clock implementation
// =============================================================================
#include "time_base.h"

#include <Arduino.h>

namespace LegBridge {

uint64_t micros64() {
  // Extend 32-bit micros() to 64 bits by counting wraps. Mirrors the platform
  // Teensy's TimeSync::micros64(). Safe as long as it is polled more often than
  // once per 71-minute wrap (every task loop qualifies).
  static uint32_t last_lo = micros();
  static uint64_t hi = 0;
  uint32_t now = micros();
  if (now < last_lo) hi += (uint64_t)1 << 32;
  last_lo = now;
  return hi | now;
}

static volatile int64_t s_wall_offset_us = 0;   // now_wall_us = micros64() + offset
static volatile bool    s_have_offset    = false;

uint64_t now_wall_us() {
  return micros64() + (uint64_t)s_wall_offset_us;
}

bool time_synced() {
  return s_have_offset;
}

void set_wall_anchor(uint64_t jetson_wall_us) {
  const int64_t local = (int64_t)micros64();
  const int64_t offset = (int64_t)jetson_wall_us - local;
  if (!s_have_offset) {
    s_wall_offset_us = offset;    // first anchor → step
    s_have_offset = true;
  } else {
    // Slew toward the new offset so the broadcast clock stays monotonic and the
    // slave IIR filters never see a backward step.
    const int64_t diff = offset - s_wall_offset_us;
    s_wall_offset_us += diff >> TIME_OFFSET_IIR_SHIFT;
  }
}

}  // namespace LegBridge
