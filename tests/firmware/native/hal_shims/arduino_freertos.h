#pragma once
// =============================================================================
//  arduino_freertos.h — HOST shim for <arduino_freertos.h>
// =============================================================================
//  freertos_shim.h does `#include <arduino_freertos.h>` to pull the
//  tsandmann/freertos-teensy umbrella (FreeRTOS.h + task.h + Arduino glue +
//  the PRIMASK intrinsics). The compiled TUs (fault_machine.cpp, leg_interp.cpp)
//  make NO FreeRTOS calls — they only need the PRIMASK intrinsics, which the
//  Arduino.h shim provides — so pulling that shim is all this needs to do.
//
//  udp_link.cpp DOES use one FreeRTOS primitive: a recursive mutex (its NetLock).
//  The fakes below stand in for it host-side and thread the take/give through the
//  net_lock_probe depth counter so the udp_link native test can assert NetLock
//  coverage. All other TUs make no FreeRTOS calls, so these are unused there.
//
//  The second of the two shim headers. See tests/firmware/native/README.md.
// =============================================================================

#include <cstdint>

#include "Arduino.h"
#include "net_lock_probe.h"

// ── FreeRTOS types / recursive-mutex fakes (host) ────────────────────────────
typedef void*    SemaphoreHandle_t;
typedef long     BaseType_t;
typedef uint32_t TickType_t;

#ifndef portMAX_DELAY
#define portMAX_DELAY ((TickType_t)0xffffffffUL)
#endif
#ifndef pdTRUE
#define pdTRUE  ((BaseType_t)1)
#endif
#ifndef pdFALSE
#define pdFALSE ((BaseType_t)0)
#endif

// A single non-null handle is enough: udp_link creates exactly one recursive
// mutex (s_net_mtx) and the depth lives in net_lock_probe, not in the handle.
static inline SemaphoreHandle_t xSemaphoreCreateRecursiveMutex() {
  return (SemaphoreHandle_t)1;
}
static inline BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t, TickType_t) {
  ++netlockprobe::depth();
  return pdTRUE;
}
static inline BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t) {
  --netlockprobe::depth();
  return pdTRUE;
}
