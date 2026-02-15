#pragma once
#include <Arduino.h>

/**
 * micros64() - 64-bit microsecond timestamp that handles 32-bit rollover.
 *
 * The built-in micros() wraps every ~71.6 minutes (2^32 µs).  This function
 * extends it to 64 bits by detecting wraps via a static high-word counter.
 *
 * ISR safety: This function is called from both ISR context (YawAxis control
 * loop) and normal context (CanInterface time sync, heartbeat, etc.).  Because
 * the two contexts now share the same static locals, interrupts are briefly
 * disabled to prevent a race on the rollover counter.
 */
inline uint64_t micros64() {
  __disable_irq();
  static uint32_t last_lo = 0;
  static uint64_t hi = 0;
  uint32_t now = ::micros();
  if (now < last_lo) hi += (uint64_t)1 << 32;
  last_lo = now;
  uint64_t result = hi | now;
  __enable_irq();
  return result;
}
