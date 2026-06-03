#pragma once
// =============================================================================
//  time_base.h — monotonic + wall-clock time for the can-bridge (time master)
// =============================================================================
//  The can-bridge Teensy is the system time-sync MASTER (teensy-can-offload.md
//  §"Time-sync master"). It owns the real-time *rate* via the on-chip crystal;
//  the Jetson is the wall-clock *anchor*, queried once at boot and every ~30 s
//  over UDP (RpcMethod::TIME_OF_DAY_QUERY) to correct long-term drift.
//
//  micros64() is the 64-bit monotonic base (the 32-bit micros() wraps at 71 min;
//  this extends it, mirroring the platform Teensy's TimeSync::micros64()).
//  now_wall_us() = micros64() + wall_offset_us. The Jetson anchor sets the
//  offset via set_wall_anchor(); subsequent corrections slew via an IIR so the
//  master clock never steps backwards (which would corrupt slave IIR filters).
// =============================================================================

#include <cstdint>
#include <Arduino.h>            // IntervalTimer, micros(), millis(), __disable_irq macro (imxrt.h)
#include "freertos_shim.h"      // tsandmann/freertos-teensy ships __get_PRIMASK / __set_PRIMASK
                                // (portmacro.h, pulled in via arduino_freertos.h).
                                // Teensy's own CMSIS tree is truncated (missing cmsis_version.h),
                                // so we get these intrinsics from the FreeRTOS port instead.

namespace CanBridge {

// 64-bit values shared between the 500 Hz IntervalTimer ISR and FreeRTOS tasks
// are NOT atomically accessible on the 32-bit Cortex-M7 (a read/write splits into
// two 32-bit ops that the ISR can interleave → torn value). These IRQ-guarded
// accessors make such access atomic. PRIMASK is saved/restored so they are safe
// to call from inside an ISR (or an already-masked region) too.
static inline uint64_t atomic_read_u64(const volatile uint64_t* p) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const uint64_t v = *p;
  __set_PRIMASK(pm);
  return v;
}
static inline void atomic_write_u64(volatile uint64_t* p, uint64_t v) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  *p = v;
  __set_PRIMASK(pm);
}

// 64-bit monotonic microseconds since boot (wrap-safe).
uint64_t micros64();

// Current wall-clock estimate (us since Unix epoch). Equals micros64() until an
// anchor is set; thereafter micros64() + wall_offset_us.
uint64_t now_wall_us();

// True once the Jetson has provided at least one wall-clock anchor.
bool time_synced();

// Apply a wall-clock anchor from the Jetson (RpcMethod::TIME_OF_DAY_QUERY result),
// captured close to when the request was sent. `jetson_wall_us` is the Jetson's
// CLOCK_REALTIME in microseconds. First call steps; later calls slew via IIR
// (shift = TIME_OFFSET_IIR_SHIFT) so the broadcast clock is monotonic.
void set_wall_anchor(uint64_t jetson_wall_us);

// IIR slew gain for drift correction: new_offset += diff >> shift.
constexpr uint8_t TIME_OFFSET_IIR_SHIFT = 4;

}  // namespace CanBridge
