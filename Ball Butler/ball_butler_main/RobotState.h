#pragma once
#include <Arduino.h>
#include "protocol_config.h"

// --------------------------------------------------------------------
// State enumeration
// --------------------------------------------------------------------
// State enumeration — values derived from YAML-generated BallButlerState constants
enum class RobotState : uint8_t {
  BOOT          = BallButlerState::BOOT,          // Homing and initialization
  IDLE          = BallButlerState::IDLE,           // Ready and waiting for command
  TRACKING      = BallButlerState::TRACKING,       // Following target
  THROWING      = BallButlerState::THROWING,       // Executing throw
  RELOADING     = BallButlerState::RELOADING,      // Grabbing next ball
  CALIBRATING   = BallButlerState::CALIBRATING,    // Calibrating location
  CHECKING_BALL = BallButlerState::CHECKING_BALL,  // Verifying ball presence after suspected removal
  ERROR         = BallButlerState::ERROR           // Error state
};

constexpr uint8_t robotStateToUint8(RobotState s) {
    return static_cast<uint8_t>(s);
}