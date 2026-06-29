#pragma once
// =============================================================================
//  Arduino.h — HOST shim for the native firmware test harness
// =============================================================================
//  Stands in for the Teensy core <Arduino.h> when the safety-critical firmware
//  (fault_machine.cpp, leg_interp.cpp) is compiled on the build host (x86_64 /
//  aarch64) and tested as a real binary. Provides ONLY what those two TUs and
//  their includes reach:
//
//    * the PRIMASK / IRQ intrinsics (used by time_base.h's atomic_*_u64 and by
//      leg_interp.cpp's setpoint publish) — no-ops on the host (no preemptible
//      ISR vs task race off-target; the harness validates LOGIC, not concurrency);
//    * micros()/millis() (declared for completeness; the compiled TUs use the
//      faked now_wall_us()/micros64() from time_base.h, not these);
//    * an IntervalTimer stub — leg_interp.cpp instantiates one and calls begin();
//      host-side the test drives interp_isr() directly (it #includes the .cpp), so
//      the timer never fires.
//
//  This is one of the two ~10-line shim headers the design proved sufficient
//  (the other is arduino_freertos.h). See tests/firmware/native/README.md.
// =============================================================================

#include <cstdint>

// ── Cortex-M PRIMASK / IRQ intrinsics → host no-ops ──────────────────────────
static inline uint32_t __get_PRIMASK() { return 0; }
static inline void     __disable_irq() {}
static inline void     __set_PRIMASK(uint32_t) {}

// ── Arduino time fns (declared; unused by the compiled TUs) ──────────────────
unsigned long micros();
unsigned long millis();

// ── IntervalTimer stub (leg_interp.cpp instantiates one; never driven host-side)
class IntervalTimer {
 public:
  bool begin(void (*)(), unsigned int) { return true; }
  void priority(unsigned char) {}
  void end() {}
};
