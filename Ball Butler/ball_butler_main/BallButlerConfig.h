#pragma once
/**
 * BallButlerConfig.h - Centralised hardware & firmware configuration
 *
 * Single source of truth for all values that change with hardware revisions,
 * motor swaps, mechanical dimensions, tuning, or firmware configuration.
 * ODrive protocol constants (command opcodes, control-mode enums, etc.) are
 * NOT included here — they live in the modules that use them.
 *
 * SECTIONS
 *   1.  ODrive Axis States       (protocol, unlikely to change)
 *   2.  CAN Node IDs
 *   3.  Hardware Pins
 *   4.  CAN Bus Configuration
 *   5.  Ball Butler CAN Protocol IDs
 *   6.  Yaw Axis Defaults
 *   7.  Pitch Axis Defaults
 *   8.  Hand Axis / Homing Defaults
 *   9.  Trajectory & Physics
 *   10. State Machine Defaults
 *   11. Operational Constants
 *   12. Heartbeat Encoding
 */

#include <cstdint>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// 1. ODrive Axis States
//    Match the ODrive CAN protocol enum (AxisState).
// ============================================================================
namespace ODriveState {
  constexpr uint32_t IDLE        = 1u;
  constexpr uint32_t CLOSED_LOOP = 8u;
}

// ============================================================================
// 2. CAN Node IDs
//    Each ODrive axis is assigned a unique node ID via the ODrive web GUI.
// ============================================================================
namespace NodeId {
  constexpr uint8_t PITCH = 7;
  constexpr uint8_t HAND  = 8;
}

// ============================================================================
// 3. Hardware Pins
// ============================================================================
namespace Pins {
  constexpr uint8_t YAW_PWM_A = 10;   // Yaw motor H-bridge PWM pin A
  constexpr uint8_t YAW_PWM_B = 16;   // Yaw motor H-bridge PWM pin B
  constexpr uint8_t YAW_ENC_A = 15;   // Yaw incremental encoder channel A
  constexpr uint8_t YAW_ENC_B = 14;   // Yaw incremental encoder channel B
}

// ============================================================================
// 4. CAN Bus Configuration
// ============================================================================
namespace CanCfg {
  constexpr uint32_t BAUD_RATE              = 1'000'000;  // 1 Mbps
  constexpr uint32_t HEARTBEAT_RATE_MS      = 100;        // BB heartbeat interval (ms)
  constexpr uint32_t AUTO_CLEAR_BRAKE_MS    = 500;        // Min ms between brake-resistor auto-clear
  constexpr uint8_t  MAX_NODES              = 16;         // Max CAN node IDs tracked
}

// ============================================================================
// 5. Ball Butler CAN Protocol IDs
//    Custom message IDs for Ball Butler <-> host communication.
// ============================================================================
namespace CanIds {
  constexpr uint32_t HOST_THROW_CMD    = 0x7D0;  // Host -> BB throw command
  constexpr uint32_t HEARTBEAT_CMD     = 0x7D1;  // BB heartbeat (BB -> Host)
  constexpr uint32_t RELOAD_CMD        = 0x7D2;  // Host -> BB reload command
  constexpr uint32_t RESET_CMD         = 0x7D3;  // Host -> BB reset command
  constexpr uint32_t CALIBRATE_LOC_CMD = 0x7D4;  // Host -> BB calibrate location
  constexpr uint32_t TIME_SYNC_CMD     = 0x7DD;  // Wall clock sync from master
}

// ============================================================================
// 6. Yaw Axis Defaults
// ============================================================================
namespace YawDefaults {
  // Encoder & motor
  constexpr uint16_t ENC_CPR          = 60;       // Incremental encoder counts per revolution
  constexpr uint32_t PWM_FREQ_HZ      = 20000;    // Motor PWM frequency (Hz)
  constexpr float    LOOP_HZ          = 150.0f;   // Control loop frequency (Hz)
  constexpr int8_t   ENC_DIR          = -1;        // Encoder direction sign
  constexpr int8_t   MOTOR_DIR        = +1;        // Motor direction sign

  // PID gains
  constexpr float KP                  = 300.0f;   // Proportional gain
  constexpr float KI                  = 0.0f;     // Integral gain
  constexpr float KD                  = 0.0f;     // Derivative gain
  constexpr float FF_PWM              = 10.0f;    // Friction feedforward (PWM counts)
  constexpr float POS_TOL_REV         = 0.0014f;  // Position tolerance (~0.5 deg)

  // Motion limits
  constexpr float ACCEL_PPS           = 500.0f;   // Acceleration limit (PWM counts/s)
  constexpr float DECEL_PPS           = 450.0f;   // Deceleration limit (PWM counts/s)
  constexpr float LIM_MIN_DEG         = 0.0f;     // Default soft limit minimum (deg)
  constexpr float LIM_MAX_DEG         = 120.0f;   // Default soft limit maximum (deg)
  constexpr float HARD_LIMIT_OVERSHOOT_DEG = 3.0f; // Hard-limit overshoot before fault (deg)

  // Velocity filtering
  constexpr float VEL_LPF_ALPHA       = 0.3f;     // Velocity low-pass filter coefficient (0–1)
  constexpr float MAX_VALID_VEL_RPS   = 2.0f;     // Glitch rejection threshold (rev/s)

  // Boot-time configuration (applied in setup() before StateMachine takes over)
  constexpr float BOOT_ZERO_OFFSET_DEG      = 5.0f;    // Encoder degrees = user 0 deg
  constexpr float BOOT_SOFT_LIM_MIN_DEG     = -5.0f;   // Wide limits during boot
  constexpr float BOOT_SOFT_LIM_MAX_DEG     = 190.0f;
  constexpr float BOOT_HARD_LIMIT_OVERSHOOT = 10.0f;   // Overshoot tolerance during boot
}

// ============================================================================
// 7. Pitch Axis Defaults
// ============================================================================
namespace PitchDefaults {
  constexpr float DEG_MIN            = 12.0f;    // Min allowed pitch angle (deg from horizontal)
  constexpr float DEG_MAX            = 90.0f;    // Max allowed pitch angle (vertical)

  // Default trapezoidal trajectory limits
  constexpr float TRAJ_VEL_RPS      = 1.0f;     // Max trap vel (rev/s)
  constexpr float TRAJ_ACCEL_RPS2   = 0.5f;     // Acceleration (rev/s^2)
  constexpr float TRAJ_DECEL_RPS2   = 0.25f;    // Deceleration (rev/s^2)
}

// ============================================================================
// 8. Hand Axis / Homing Defaults
// ============================================================================
namespace HandDefaults {
  // homeHandStandard() defaults
  constexpr int   HOMING_DIRECTION      = -1;       // Direction during homing (-1 = retract)
  constexpr float HOMING_SPEED_RPS      = 3.0f;     // Homing speed (rev/s)
  constexpr float HOMING_CURRENT_A      = 5.0f;     // Motor current limit during homing (A)
  constexpr float HOMING_HEADROOM_A     = 3.0f;     // Current headroom for spike detection (A)
  constexpr float HOMING_ABS_POS_REV    = -0.1f;    // Absolute position after homing (rev)

  // homeHand() defaults
  constexpr float HOMING_EMA_WEIGHT     = 0.7f;     // EMA weight for Iq spike detection
  constexpr uint16_t HOMING_IQ_POLL_MS  = 10;       // Iq polling interval during homing (ms)
  constexpr uint32_t HOMING_ATTEMPT_TIMEOUT_MS = 5000;  // Timeout for a single homing attempt (ms)
  constexpr uint32_t HOMING_MODE_SETTLE_MS     = 100;   // Delay after setting control mode before sending homing velocity (ms)
  constexpr uint32_t HOMING_STOP_SETTLE_MS     = 5;     // Delay after stopping motor before setting absolute position (ms)

  // restoreHandToOperatingConfig() defaults
  constexpr float OP_VEL_LIMIT_RPS      = 1000.0f;  // Operating velocity limit (rev/s)
  constexpr float OP_CURRENT_LIMIT_A    = 50.0f;    // Operating current limit (A)
}

// ============================================================================
// 9. Trajectory & Physics
// ============================================================================
namespace TrajCfg {
  // Physical properties
  constexpr float G                      = 9.806f;      // Gravitational acceleration (m/s^2)
  constexpr float HAND_SPOOL_R           = 0.0052493f;  // Hand spool radius (m)
  constexpr float LINEAR_GAIN_FACTOR     = 1.0f;        // Multiplier on linear gain (reserved)
  constexpr float LINEAR_GAIN            = LINEAR_GAIN_FACTOR / (M_PI * HAND_SPOOL_R * 2.f);  // rev/m
  constexpr float INERTIA_HAND_ONLY      = 0.281f;      // Hand inertia (kg)
  constexpr float INERTIA_RATIO          = 0.747f;      // Inertia ratio for throw dynamics
  constexpr float HAND_STROKE            = 0.28f;       // Total hand linear stroke (m)
  constexpr float STROKE_MARGIN          = 0.02f;       // Safety margin at ends of stroke (m)

  // Trajectory tuning
  constexpr float THROW_VEL_HOLD_PCT     = 0.05f;       // Fraction of throw at constant velocity
  constexpr float CATCH_VEL_RATIO        = 0.8f;        // Catch velocity as fraction of throw velocity
  constexpr float CATCH_VEL_HOLD_PCT     = 0.10f;       // Fraction of catch at constant velocity
  constexpr float END_PROFILE_HOLD       = 0.10f;       // Hold time at end of motion profile (s)
  constexpr int   SAMPLE_RATE            = 500;          // Trajectory sample rate (Hz)

  // Smooth move
  constexpr float MAX_SMOOTH_ACCEL       = 200.0f;      // Max hand accel for smooth moves (rev/s^2)
  constexpr float QUINTIC_S2_MAX         = 5.7735027f;  // Max |s''| for quintic polynomial
  constexpr float HAND_MAX_SMOOTH_POS    = 8.9f;        // Max hand position for smooth moves (rev)
}

// ============================================================================
// 10. State Machine Defaults
//     These provide default values for StateMachine::Config members.
// ============================================================================
namespace SMDefaults {
  // Timeouts
  constexpr uint32_t HOMING_TIMEOUT_MS            = 10000;  // Max time for homing
  constexpr uint32_t HOMING_RETRY_DELAY_MS        = 500;    // Delay between homing retries
  constexpr uint32_t RELOAD_TIMEOUT_MS            = 10000;  // Max time for reload sequence
  constexpr uint32_t POST_THROW_DELAY_MS          = 1000;   // Delay after throw before reload
  constexpr uint32_t IDLE_NO_CMD_TIMEOUT_MS       = 5000;   // Stow pitch if idle this long

  // Reload sequence positions
  constexpr float RELOAD_HAND_TOP_REV             = 8.7f;
  constexpr float RELOAD_PITCH_READY_DEG          = 70.0f;
  constexpr float RELOAD_YAW_ANGLE_DEG            = 180.0f;
  constexpr float RELOAD_PITCH_GRAB_DEG           = 90.0f;
  constexpr float RELOAD_HAND_BOTTOM_REV          = 0.0f;
  constexpr float RELOAD_HAND_BOTTOM_TOL_REV      = 0.1f;
  constexpr uint32_t RELOAD_HOLD_DELAY_MS         = 500;
  constexpr uint8_t  RELOAD_BALL_CHECK_SAMPLES    = 5;

  // Tracking & rate limiting
  constexpr uint32_t TRACKING_HAND_CHECK_MS       = 200;
  constexpr uint32_t YAW_CMD_INTERVAL_MS          = 10;
  constexpr uint32_t PITCH_CMD_INTERVAL_MS        = 50;

  // Ball detection (CHECKING_BALL state)
  constexpr uint8_t CHECK_BALL_CONFIRM_SAMPLES    = 5;
  constexpr float   CHECK_BALL_DISRUPT_PITCH_DEG  = 70.0f;

  // Home positions
  constexpr float HAND_REV_HOME                   = 0.0f;
  constexpr float PITCH_DEG_HOME                  = 90.0f;
  constexpr float YAW_DEG_HOME                    = 20.0f;

  // Calibration
  constexpr float CALIB_MAX_YAW_DEG               = 120.0f;
  constexpr float CALIB_MIN_YAW_DEG               = 0.0f;
  constexpr float CALIB_YAW_ACCEL                 = 50.0f;
  constexpr float CALIB_YAW_DECEL                 = 50.0f;
  constexpr uint32_t CALIB_PAUSE_MS               = 2500;

  // Retry limits
  constexpr uint8_t MAX_RELOAD_ATTEMPTS           = 3;
  constexpr uint8_t MAX_HOMING_ATTEMPTS           = 3;

  // Settle times & thresholds
  constexpr uint32_t PITCH_SETTLE_MS              = 500;
  constexpr uint32_t PITCH_GRAB_SETTLE_MS         = 1000;
  constexpr float    YAW_ANGLE_THRESHOLD_DEG      = 1.0f;

  // Motion timing
  constexpr uint32_t PITCH_IDLE_DELAY_MS          = 500;    // Min ms after last pitch command before allowing IDLE mode
  constexpr uint32_t MIN_MOTION_DURATION_MS       = 300;    // Min ms to wait before checking trajectory_done (allows ODrive to begin movement)
  constexpr uint32_t PV_FRESHNESS_US              = 20000;  // Max age of position/velocity data (µs) before considered stale

  // Axis limits
  constexpr float PITCH_MIN_STOW_DEG              = 80.0f;
  constexpr float YAW_MIN_ANGLE_DEG               = 5.0f;
  constexpr float YAW_MAX_ANGLE_DEG               = 185.0f + YAW_MIN_ANGLE_DEG;
}

// ============================================================================
// 11. Operational Constants
// ============================================================================
namespace OpCfg {
  constexpr float    THROW_VEL_MIN_MPS   = 0.05f;    // Min allowed throw velocity (m/s)
  constexpr float    THROW_VEL_MAX_MPS   = 6.0f;     // Max allowed throw velocity (m/s)
  constexpr float    SCHEDULE_MARGIN_S   = 0.1f;     // Throw scheduling time margin (s). Ensures enough lead time for trajectory generation and state machine transitions.
  constexpr uint16_t YAW_TELEM_MS        = 500;      // Yaw telemetry streaming interval (ms)
  constexpr uint32_t SERIAL_BAUD         = 115200;   // USB serial speed
  constexpr uint32_t SERIAL_WAIT_MS      = 2000;     // Max ms to wait for Serial at boot
  constexpr uint32_t ODRIVE_BOOT_MS      = 1000;     // Delay after CAN init for ODrives (ms)
}

// ============================================================================
// 12. Heartbeat Encoding
//     Resolution and range values for the 8-byte heartbeat CAN frame.
// ============================================================================
namespace HeartbeatCfg {
  constexpr float YAW_RES_DEG      = 0.01f;     // Yaw encoding resolution (deg/count)
  constexpr float PITCH_RES_DEG    = 0.002f;     // Pitch encoding resolution (deg/count)
  constexpr float HAND_RES_MM      = 0.01f;      // Hand encoding resolution (mm/count)
  constexpr float HAND_MAX_MM      = 655.36f;    // Max representable hand position (mm)
  constexpr float PITCH_CLAMP_MIN  = 0.0f;       // Pitch encoding min (deg)
  constexpr float PITCH_CLAMP_MAX  = 90.0f;      // Pitch encoding max (deg)
}
