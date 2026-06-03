#pragma once
// =============================================================================
//  freertos_shim.h — single include point for the FreeRTOS Teensy 4 port
// =============================================================================
//  The parent plan targets the FreeRTOS_TEENSY4 port. The originally-cited
//  greiman/FreeRTOS_TEENSY4 repo has been deleted (404 as of 2026-06-02); the
//  actively-maintained successor is tsandmann/freertos-teensy:
//
//    * tsandmann/freertos-teensy  →  #include <arduino_freertos.h>  (this)
//
//  The umbrella header pulls in FreeRTOS.h + task.h + Arduino glue. The
//  standard FreeRTOS C API (xTaskCreate, vTaskDelay, vTaskDelayUntil,
//  xSemaphore*, queues) is in the global namespace and is fork-independent.
//
//  Pin the exact library + version in README.md once the bench build is green
//  (PlatformIO library.json declares version 11.2.0-3 at the time of writing).
// =============================================================================

#include <arduino_freertos.h>
