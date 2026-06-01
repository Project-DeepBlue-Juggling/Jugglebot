#pragma once
// =============================================================================
//  freertos_shim.h — single include point for the FreeRTOS Teensy 4 port
// =============================================================================
//  The parent plan targets the FreeRTOS_TEENSY4 port. Two common forks exist
//  and differ in their umbrella header:
//
//    * greiman/FreeRTOS_TEENSY4        →  #include "FreeRTOS_TEENSY4.h"   (this)
//    * tsandmann/freertos-teensy       →  #include "arduino_freertos.h"   (namespaced)
//
//  We standardise on the greiman header (its name matches the plan's
//  "FreeRTOS_TEENSY4" verbatim). If the tsandmann fork is used instead, change
//  the include below and add `using namespace arduino_freertos;`. The rest of
//  the firmware uses only the standard FreeRTOS API (xTaskCreate, vTaskDelay,
//  vTaskDelayUntil, xSemaphore*, queues), which is identical across forks.
//
//  Pin the exact library + version in README.md.
// =============================================================================

#include "FreeRTOS_TEENSY4.h"
