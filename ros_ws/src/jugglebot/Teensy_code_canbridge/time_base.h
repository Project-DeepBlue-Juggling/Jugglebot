#pragma once
// =============================================================================
//  time_base.h — monotonic + wall-clock time for the can-bridge (time master)
// =============================================================================
//  The can-bridge Teensy is the system time-sync MASTER. It owns the real-time
//  *rate* via the on-chip crystal;
//  the Jetson is the wall-clock *anchor*, queried once at boot and every ~30 s
//  over UDP (RpcMethod::TIME_OF_DAY_QUERY) to correct long-term drift.
//
//  micros64() is the 64-bit monotonic base (the 32-bit micros() wraps at 71 min;
//  this extends it, mirroring the platform Teensy's TimeSync::micros64()).
//  now_wall_us() = micros64() + wall_offset_us. The Jetson anchor sets the
//  offset via set_wall_anchor(): small drift is slewed via an IIR (monotonic),
//  but a LARGE error (boot / re-acquisition after a sync gap) is stepped — a slew
//  there would take ~30 min and masquerades as a thrower "warm-up" drift in the
//  cone (logbook/2026-06-12-temporal-warmup-drift.md).
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
//
// CLOCK-DISCIPLINE INVARIANT: use micros64() for ALL interval /
// staleness / age arithmetic (heartbeat freshness, link timeouts, watchdog
// deadlines, the 500 Hz trajectory phase, cone/bus gates, diag ages). micros64()
// is the pure crystal that set_wall_anchor() NEVER touches, so an NTP-style wall
// STEP (forward or backward) cannot corrupt any interval or underflow a uint64
// age. now_wall_us() is reserved for WIRE-BOUND absolute timestamps only — a
// value serialised onto the CAN/UDP wire and interpreted against another node's
// clock (t_teensy_us / t_bridge_us fields, the 0x7DD time-sync broadcast, the
// Ball Butler throw_time). Never compute an interval as a difference of two
// now_wall_us() reads that straddle a possible anchor step.
uint64_t micros64();

// Current wall-clock estimate (us since Unix epoch). Equals micros64() until an
// anchor is set; thereafter micros64() + wall_offset_us. STEPPABLE by
// set_wall_anchor() — use ONLY for wire-bound absolute timestamps (see the
// clock-discipline invariant on micros64() above); never for intervals.
uint64_t now_wall_us();

// True when the Jetson anchor is present AND fresh (a recent TOD response). Goes
// false after TIME_ANCHOR_STALE_US without a new anchor (e.g. teensy_bridge_node
// down) so the broadcast self-gates and slaves learn the clock is untrustworthy.
bool time_synced();

// Apply a wall-clock anchor from the Jetson (RpcMethod::TIME_OF_DAY_QUERY result),
// captured close to when the request was sent. `jetson_wall_us` is the Jetson's
// CLOCK_REALTIME in microseconds. STEPS on the first anchor and whenever the
// error exceeds TIME_STEP_THRESHOLD_US (boot / re-acquisition after a sync gap —
// the bridge free-runs and drifts while teensy_bridge_node is down); otherwise
// SLEWS via the IIR for smooth, monotonic small-drift correction.
void set_wall_anchor(uint64_t jetson_wall_us);

// ── Per-anchor clock-discipline diagnostics (FW 11, CLOCK_DIAG 0x8F) ─────────
//  set_wall_anchor() has always computed the pre-slew offset error and thrown it
//  away, so the ONE quantity that says how well this clock is actually
//  disciplined has never left the chip. This captures it, plus enough context to
//  make each sample self-contained, so a bag can answer the questions
//  plans/active/bridge-clock-frequency-discipline.md Phase 1 asks (what IS the
//  crystal's ppm, how does it move with temperature, where is the RTT-jitter
//  floor) BEFORE the servo is redesigned around a guess.
//
//  Diagnostics ONLY. Nothing in the clock or the interp reads these fields, so a
//  wrong value here cannot move a leg — which is why the capture is allowed to
//  sit in set_wall_anchor at all.
struct ClockAnchorSample {
  uint64_t t_local_us;       // micros64() at the anchor (the pure-crystal x-axis)
  uint64_t jetson_wall_us;   // the RTT-corrected anchor value actually applied
  uint32_t dt_local_us;      // micros64 since the previous accepted anchor (0 on the first);
                             // saturates at UINT32_MAX (~71.6 min)
  int32_t  err_us;           // pre-slew offset error (the discarded `diff`), saturated to i32;
                             // 0 and meaningless when FIRST_ANCHOR is set
  int32_t  freq_ppb;         // implied crystal frequency error vs the Jetson over dt_local_us;
                             // 0 and meaningless unless FREQ_VALID is set
  uint32_t anchor_seq;       // accepted anchors since boot (1 on the first) — the frame-loss key
  uint8_t  flags;            // JbUdp::ClockDiagFlags (STEPPED / FIRST_ANCHOR / FREQ_VALID)
};

// Snapshot the most recent accepted anchor's diagnostics into *out. Returns false
// (leaving *out untouched) until the first anchor has landed. Safe from any task:
// the copy is IRQ-masked, because the struct is many words wide and the writer
// (the net task, inside set_wall_anchor) can preempt a reader mid-copy and hand
// it a mix of two anchors — which in a ppm fit is not noise, it is a fabricated
// data point.
bool clock_last_anchor(ClockAnchorSample* out);

// IIR slew gain for small-drift correction: new_offset += diff >> shift.
constexpr uint8_t TIME_OFFSET_IIR_SHIFT = 4;

// Step (don't slew) when |anchor − current offset| exceeds this. Slewing a large
// re-acquisition error at 1/16-every-30 s takes ~30+ min (the 2026-06-16 cone
// "warm-up" artifact); a step is bounded and slaves re-lock in <1 s. Only fires
// at boot / after a sync gap — normal 30 s inter-anchor drift is <2 ms.
constexpr int64_t TIME_STEP_THRESHOLD_US = 20'000;        // 20 ms

// time_synced() goes false this long after the last anchor (≈3× the 30 s resync)
// so a lost Jetson link self-reports as unsynced instead of silently drifting.
constexpr uint64_t TIME_ANCHOR_STALE_US = 90'000'000ULL;  // 90 s

}  // namespace CanBridge
