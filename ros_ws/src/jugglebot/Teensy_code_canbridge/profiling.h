#pragma once
// =============================================================================
//  profiling.h — firmware instrumentation → 1 Hz PROFILE UDP frame
// =============================================================================
//  Gathers per-task CPU%, CAN1/CAN2 bus utilisation, UDP RTT/jitter, the 500 Hz
//  interp deadline-miss counter, and free heap, and emits a JbUdp::PROFILE frame
//  once per second. Consumed by tools/probes/teensy_link_profiling/jetson.
//
//  Per-task CPU requires FreeRTOSConfig:
//      configGENERATE_RUN_TIME_STATS = 1
//      configUSE_TRACE_FACILITY      = 1
//    + portCONFIGURE_TIMER_FOR_RUN_TIME_STATS / portGET_RUN_TIME_COUNTER_VALUE
//      wired to a high-res counter (DWT->CYCCNT, see profiling.cpp comment).
//  If those are absent the CPU fields report 0 (everything else still works).
//
//  PROFILE cpu_pct_x100[] slot order (shared with the Jetson consumer):
//      0:canrx 1:tsync 2:net 3:fault 4:telem 5:hb 6:diag 7:IDLE 8:other
//  (the 500 Hz interpolator runs in an ISR, not a task, so its CPU is not in the
//  task stats — its health is the interp_deadline_misses / interp_max_jitter_us
//  fields instead.)
// =============================================================================

#include <cstdint>

namespace CanBridge {

void profiling_init();
// Call at 1 Hz (DIAG_HEARTBEAT_HZ). Builds + sends the PROFILE frame and resets
// the per-window counters.
void profiling_step();

}  // namespace CanBridge
