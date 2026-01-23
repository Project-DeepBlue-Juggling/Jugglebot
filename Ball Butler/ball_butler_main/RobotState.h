#pragma once
#include <Arduino.h>

// --------------------------------------------------------------------
// State enumeration
// --------------------------------------------------------------------
enum class RobotState : uint8_t {
  BOOT        = 0,  // Homing and initialization
  IDLE        = 1,  // Ready and waiting for command
  THROWING    = 2,  // Executing throw
  RELOADING   = 3,  // Grabbing next ball
  CALIBRATING = 4, // Calibrating location
  ERROR       = 127   // Error state
};

constexpr uint8_t robotStateToUint8(RobotState s) {
    return static_cast<uint8_t>(s);
}