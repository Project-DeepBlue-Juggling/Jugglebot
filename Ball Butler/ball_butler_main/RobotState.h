#pragma once
#include <Arduino.h>

// --------------------------------------------------------------------
// State enumeration
// --------------------------------------------------------------------
enum class RobotState : uint8_t {
  BOOT          = 0,  // Homing and initialization
  IDLE          = 1,  // Ready and waiting for command
  TRACKING      = 2,  // Following target
  THROWING      = 3,  // Executing throw
  RELOADING     = 4,  // Grabbing next ball
  CALIBRATING   = 5,  // Calibrating location
  CHECKING_BALL = 6,  // Verifying ball presence after suspected removal
  ERROR         = 127 // Error state
};

constexpr uint8_t robotStateToUint8(RobotState s) {
    return static_cast<uint8_t>(s);
}