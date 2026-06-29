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
//  The second of the two shim headers. See tests/firmware/native/README.md.
// =============================================================================

#include "Arduino.h"
