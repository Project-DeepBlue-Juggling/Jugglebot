// PitchAxis.h
#pragma once
#include <Arduino.h>
#include "CanInterface.h"

/**
 * PitchAxis
 *
 * Controls the pitch axis driven by an ODrive Micro over CAN.
 * - Position units on the drive are **revolutions** of the load encoder.
 * - User commands in **degrees from the horizontal**, where:
 *      90.0 deg  ->  0.00 rev   (vertical)
 *      10.8 deg  -> -0.22 rev   (lowest)
 *   Mapping is linear:  rev = (deg - 90.0) / 360.0
 *
 * Operates in:
 *   CONTROL_MODE_POSITION (3) + INPUT_MODE_TRAP_TRAJ (5)
 *   with CLOSED_LOOP_CONTROL (8).
 *
 * Robust input handling:
 *   - Rejects angles outside [10.8, 90.0] deg with a clear Serial message.
 *   - Will not send set_input_pos if the axis isn’t homed (gate lives in CanInterface).
 */

class PitchAxis {
public:
  // Allowed user command range in DEGREES
  static constexpr float DEG_MIN = 12.0f; // Actual min is ~10.8, but let's have a bit of margin for now...
  static constexpr float DEG_MAX = 90.0f;

  // Corresponding REV range (derived, for reference)
  static constexpr float REV_MIN = (DEG_MIN - DEG_MAX) / 360.0f;
  static constexpr float REV_MAX = 0.0f;

  // Default trajectory limits (turns/s, turns/s^2)
  struct Traj {
    float vel_limit_rps  = 1.0f;    // max trap vel (rev/s)
    float accel_rps2     = 0.5f;    // accel
    float decel_rps2     = 0.25f;   // decel
  };

  PitchAxis(CanInterface& can, uint8_t node_id, Stream* log = &Serial);

  // Call once after CAN is up (e.g., from setup()).
  // Configures controller + trajectory mode and enters CLOSED_LOOP_CONTROL.
  bool begin();                          // <- overload with defaults
  bool begin(const Traj& tcfg);          // <- explicit config

  // Set a target in **degrees from horizontal** (validated).
  // Returns true if accepted & sent, false if rejected.
  bool setTargetDeg(float deg);

  // Helpers
  static inline float degToRev(float deg) { return (deg - 90.0f) * (1.0f/360.0f); }
  static inline float revToDeg(float rev) { return 90.0f + 360.0f * rev; }

  // Update trajectory limits at runtime (rps, rps^2).
  bool setTrajLimits(float vel_rps, float acc_rps2, float dec_rps2);

  // Change basic gains if you like (pass-through to ODrive CAN)
  bool setGains(float kp_pos, float kv_vel, float ki_vel);

  // Print a short status line (uses latest estimator snapshot if available)
  void printStatusOnce() const;

private:
  CanInterface& can_;
  uint8_t node_;
  Stream* log_;
  Traj traj_;
  bool configured_ = false;

  // ODrive enums
  static constexpr uint32_t AXIS_STATE_CLOSED_LOOP = 8u;
  static constexpr uint32_t CONTROL_MODE_POSITION  = 3u;
  static constexpr uint32_t INPUT_MODE_TRAP_TRAJ   = 5u;

  bool configureController_();
  bool applyTraj_(const Traj& t);
  bool enterClosedLoop_();
  bool sendTargetRev_(float target_rev);
  void reject_(const char* why, float value, float lo, float hi) const;
};
