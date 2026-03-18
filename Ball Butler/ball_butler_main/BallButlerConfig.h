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
 *   11. Ball Detection Configuration
 *   12. Operational Constants
 *   13. Heartbeat Encoding
 */

#include <cstdint>
#include <cmath>

// Shared protocol constants (auto-generated from config/protocol_config.yaml)
#include "protocol_config.h"

// Hardware & geometry constants (auto-generated from config/hardware_config.yaml)
// All namespace aliases below source values from the generated BB* namespaces
// so that YAML changes propagate automatically.
#include "hardware_config.h"

// ============================================================================
// 1. ODrive Axis States
//    Provided by protocol_config.h (namespace ODriveState)
// ============================================================================

// ============================================================================
// 2. CAN Node IDs
//    Provided by protocol_config.h (namespace NodeId).
//    Use NodeId::BB_PITCH and NodeId::BB_HAND directly.
// ============================================================================

// ============================================================================
// 3. Hardware Pins — sourced from hardware_config.h (BBPins)
// ============================================================================
namespace Pins {
  constexpr uint8_t YAW_SPI_CS = BBPins::YAW_SPI_CS;  // AS5047P absolute encoder SPI chip select
  constexpr uint8_t YAW_INA1   = BBPins::YAW_INA1;    // H-bridge direction pin INA1
  constexpr uint8_t YAW_INA2   = BBPins::YAW_INA2;    // H-bridge direction pin INA2
  constexpr uint8_t YAW_PWM    = BBPins::YAW_PWM;     // H-bridge PWM speed pin
}

namespace YawMotor {
  constexpr uint8_t PWM_DEADZONE_MIN = BBYaw::PWM_DEADZONE_MIN;
}

// ============================================================================
// 4. CAN Bus Configuration — sourced from hardware_config.h (BBCan)
// ============================================================================
namespace CanCfg {
  constexpr uint32_t BAUD_RATE              = CanBus::BAUD_RATE;               // from protocol_config.h
  constexpr uint32_t HEARTBEAT_RATE_MS      = BBCan::HEARTBEAT_RATE_MS;
  constexpr uint32_t AUTO_CLEAR_BRAKE_MS    = BBCan::AUTO_CLEAR_BRAKE_MS;
  constexpr uint8_t  MAX_NODES              = BBCan::MAX_NODES;
  constexpr uint32_t SYNC_STATS_PRINT_US    = BBCan::SYNC_STATS_PRINT_MS * 1000u;  // YAML stores ms, convert to µs
}

// ============================================================================
// 5. Ball Butler CAN Protocol IDs
//    Canonical values now in protocol_config.h (BallButlerCanId, PlatformCanId).
//    Local aliases preserve existing call-site names.
// ============================================================================
namespace CanIds {
  constexpr uint32_t HOST_THROW_CMD    = BallButlerCanId::THROW_CMD;
  constexpr uint32_t HEARTBEAT_CMD     = BallButlerCanId::HEARTBEAT;
  constexpr uint32_t RELOAD_CMD        = BallButlerCanId::RELOAD_CMD;
  constexpr uint32_t RESET_CMD         = BallButlerCanId::RESET_CMD;
  constexpr uint32_t CALIBRATE_LOC_CMD = BallButlerCanId::CALIBRATE_LOC_CMD;
  constexpr uint32_t TIME_SYNC_CMD     = SharedCanId::TIME_SYNC;
}

// ============================================================================
// 6. Yaw Axis Defaults — sourced from hardware_config.h (BBYaw)
// ============================================================================
namespace YawDefaults {
  // Encoder & motor
  constexpr uint16_t ENC_CPR          = BBYaw::ENC_CPR;
  constexpr float    ABS_ENC_CPR      = BBYaw::ABS_ENC_CPR;
  constexpr uint32_t PWM_FREQ_HZ      = BBYaw::PWM_FREQ_HZ;
  constexpr uint8_t  PWM_MAX          = BBYaw::PWM_MAX;
  constexpr float    LOOP_HZ          = BBYaw::LOOP_HZ;
  constexpr int8_t   ENC_DIR          = BBYaw::ENC_DIR;
  constexpr int8_t   MOTOR_DIR        = BBYaw::MOTOR_DIR;

  // PID gains
  constexpr float KP                  = BBYaw::KP;
  constexpr float KI                  = BBYaw::KI;
  constexpr float KD                  = BBYaw::KD;
  constexpr float FF_PWM              = BBYaw::FF_PWM;
  constexpr float FF_TAPER_DEG        = BBYaw::FF_TAPER_DEG;
  constexpr float POS_TOL_REV         = BBYaw::POS_TOL_REV;

  // Motion limits
  constexpr float ACCEL_PPS           = BBYaw::ACCEL_PPS;
  constexpr float DECEL_PPS           = BBYaw::DECEL_PPS;
  constexpr float LIM_MIN_DEG         = BBYaw::LIM_MIN_DEG;
  constexpr float LIM_MAX_DEG         = BBYaw::LIM_MAX_DEG;
  constexpr float HARD_LIMIT_OVERSHOOT_DEG = BBYaw::HARD_LIMIT_OVERSHOOT_DEG;
  constexpr float MAX_SOFT_RANGE_DEG  = BBYaw::MAX_SOFT_RANGE_DEG;

  // Velocity filtering
  constexpr float VEL_LPF_ALPHA       = BBYaw::VEL_LPF_ALPHA;
  constexpr float MAX_VALID_VEL_RPS   = BBYaw::MAX_VALID_VEL_RPS;

  // Boot-time configuration (applied in setup() before StateMachine takes over)
  constexpr float BOOT_ZERO_OFFSET_DEG      = BBYaw::BOOT_ZERO_OFFSET_DEG;
  constexpr float BOOT_SOFT_LIM_MIN_DEG     = BBYaw::BOOT_SOFT_LIM_MIN_DEG;
  constexpr float BOOT_SOFT_LIM_MAX_DEG     = BBYaw::BOOT_SOFT_LIM_MAX_DEG;
  constexpr float BOOT_HARD_LIMIT_OVERSHOOT = BBYaw::BOOT_HARD_LIMIT_OVERSHOOT_DEG;
}

// ============================================================================
// 7. Pitch Axis Defaults — sourced from hardware_config.h (BBPitch)
// ============================================================================
namespace PitchDefaults {
  constexpr float DEG_MIN            = BBPitch::DEG_MIN;
  constexpr float DEG_MAX            = BBPitch::DEG_MAX;

  // Default trapezoidal trajectory limits
  constexpr float TRAJ_VEL_RPS      = BBPitch::TRAJ_VEL_RPS;
  constexpr float TRAJ_ACCEL_RPS2   = BBPitch::TRAJ_ACCEL_RPS2;
  constexpr float TRAJ_DECEL_RPS2   = BBPitch::TRAJ_DECEL_RPS2;
}

// ============================================================================
// 8. Hand Axis / Homing Defaults — sourced from hardware_config.h (BBHand)
// ============================================================================
namespace HandDefaults {
  // homeHandStandard() defaults
  constexpr int   HOMING_DIRECTION      = BBHand::HOMING_DIRECTION;
  constexpr float HOMING_SPEED_RPS      = BBHand::HOMING_SPEED_RPS;
  constexpr float HOMING_CURRENT_A      = BBHand::HOMING_CURRENT_A;
  constexpr float HOMING_HEADROOM_A     = BBHand::HOMING_HEADROOM_A;
  constexpr float HOMING_ABS_POS_REV    = BBHand::HOMING_ABS_POS_REV;

  // homeHand() defaults
  constexpr float HOMING_EMA_WEIGHT     = BBHand::HOMING_EMA_WEIGHT;
  constexpr uint16_t HOMING_IQ_POLL_MS  = BBHand::HOMING_IQ_POLL_MS;
  constexpr uint32_t HOMING_ATTEMPT_TIMEOUT_MS = BBHand::HOMING_ATTEMPT_TIMEOUT_MS;
  constexpr uint32_t HOMING_MODE_SETTLE_MS     = BBHand::HOMING_MODE_SETTLE_MS;
  constexpr uint32_t HOMING_STOP_SETTLE_MS     = BBHand::HOMING_STOP_SETTLE_MS;

  // restoreHandToOperatingConfig() defaults
  constexpr float OP_VEL_LIMIT_RPS      = BBHand::OP_VEL_LIMIT_RPS;
  constexpr float OP_CURRENT_LIMIT_A    = BBHand::OP_CURRENT_LIMIT_A;
}

// ============================================================================
// 9. Trajectory & Physics
// ============================================================================
namespace TrajCfg {
  // Physical properties — sourced from hardware_config.h (BBTraj / Physics)
  constexpr float G                      = Physics::GRAVITY_MPS2;
  constexpr float HAND_SPOOL_R           = BBTraj::HAND_SPOOL_RADIUS_M;
  constexpr float LINEAR_GAIN_FACTOR     = BBTraj::LINEAR_GAIN_FACTOR;
  constexpr float LINEAR_GAIN            = LINEAR_GAIN_FACTOR / (M_PI * HAND_SPOOL_R * 2.f);  // rev/m (derived)
  constexpr float INERTIA_HAND_ONLY      = BBTraj::INERTIA_HAND_ONLY_KG;
  constexpr float INERTIA_RATIO          = BBTraj::INERTIA_RATIO;
  constexpr float HAND_STROKE            = BBTraj::HAND_STROKE_M;
  constexpr float STROKE_MARGIN          = BBTraj::STROKE_MARGIN_M;

  // Trajectory tuning — sourced from hardware_config.h (BBTraj)
  constexpr float THROW_VEL_HOLD_PCT     = BBTraj::THROW_VEL_HOLD_PCT;
  constexpr float CATCH_VEL_RATIO        = BBTraj::CATCH_VEL_RATIO;
  constexpr float CATCH_VEL_HOLD_PCT     = BBTraj::CATCH_VEL_HOLD_PCT;
  constexpr float END_PROFILE_HOLD       = BBTraj::END_PROFILE_HOLD_S;
  constexpr int   SAMPLE_RATE            = (int)BBTraj::SAMPLE_RATE_HZ;

  // Smooth move — sourced from hardware_config.h (BBTraj)
  constexpr float MAX_SMOOTH_ACCEL       = BBTraj::MAX_SMOOTH_ACCEL_RPS2;
  constexpr float QUINTIC_S2_MAX         = BBTraj::QUINTIC_S2_MAX;
  constexpr float HAND_MAX_SMOOTH_POS    = BBTraj::HAND_MAX_SMOOTH_POS_REV;

  // Pre-allocated trajectory buffer sizing (partially from hardware_config.h, partially derived)
  constexpr float MAX_THROW_DURATION_S   = BBTraj::MAX_THROW_DURATION_S;
  constexpr size_t MAX_THROW_SAMPLES     = (size_t)(MAX_THROW_DURATION_S * SAMPLE_RATE) + 1;  // ~1001
  constexpr size_t MAX_SMOOTH_SAMPLES    = (size_t)BBTraj::MAX_SMOOTH_SAMPLES;
  constexpr size_t MAX_PAUSE_SAMPLES     = (size_t)BBTraj::MAX_PAUSE_SAMPLES;
  constexpr size_t MAX_TRAJ_FRAMES       = MAX_THROW_SAMPLES + MAX_SMOOTH_SAMPLES + MAX_PAUSE_SAMPLES;  // ~1813
  constexpr size_t TRAJ_ARENA_BYTES      = (size_t)BBTraj::TRAJ_ARENA_BYTES;
}

// ============================================================================
// 10. State Machine Defaults — sourced from hardware_config.h (BBSM)
//     These provide default values for StateMachine::Config members.
// ============================================================================
namespace SMDefaults {
  // Timeouts
  constexpr uint32_t HOMING_TIMEOUT_MS            = BBSM::HOMING_TIMEOUT_MS;
  constexpr uint32_t HOMING_RETRY_DELAY_MS        = BBSM::HOMING_RETRY_DELAY_MS;
  constexpr uint32_t RELOAD_TIMEOUT_MS            = BBSM::RELOAD_TIMEOUT_MS;
  constexpr uint32_t POST_THROW_DELAY_MS          = BBSM::POST_THROW_DELAY_MS;
  constexpr uint32_t IDLE_NO_CMD_TIMEOUT_MS       = BBSM::IDLE_NO_CMD_TIMEOUT_MS;

  // Reload sequence positions
  constexpr float RELOAD_HAND_TOP_REV             = BBSM::RELOAD_HAND_TOP_REV;
  constexpr float RELOAD_PITCH_READY_DEG          = BBSM::RELOAD_PITCH_READY_DEG;
  constexpr float RELOAD_YAW_ANGLE_DEG            = BBSM::RELOAD_YAW_ANGLE_DEG;
  constexpr float RELOAD_PITCH_GRAB_DEG           = BBSM::RELOAD_PITCH_GRAB_DEG;
  constexpr float RELOAD_HAND_BOTTOM_REV          = BBSM::RELOAD_HAND_BOTTOM_REV;
  constexpr float RELOAD_HAND_BOTTOM_TOL_REV      = BBSM::RELOAD_HAND_BOTTOM_TOL_REV;
  constexpr uint32_t RELOAD_HOLD_DELAY_MS         = BBSM::RELOAD_HOLD_DELAY_MS;
  constexpr uint8_t  RELOAD_BALL_CHECK_SAMPLES    = BBSM::RELOAD_BALL_CHECK_SAMPLES;
  constexpr uint32_t BALL_CHECK_SAMPLE_INTERVAL_MS = BBSM::BALL_CHECK_SAMPLE_INTERVAL_MS;

  // Tracking & rate limiting
  constexpr uint32_t TRACKING_HAND_CHECK_MS       = BBSM::TRACKING_HAND_CHECK_MS;
  constexpr uint32_t YAW_CMD_INTERVAL_MS          = BBSM::YAW_CMD_INTERVAL_MS;
  constexpr uint32_t PITCH_CMD_INTERVAL_MS        = BBSM::PITCH_CMD_INTERVAL_MS;

  // Ball detection (CHECKING_BALL state)
  constexpr uint8_t CHECK_BALL_CONFIRM_SAMPLES    = BBSM::CHECK_BALL_CONFIRM_SAMPLES;
  constexpr float   CHECK_BALL_DISRUPT_PITCH_DEG  = BBSM::CHECK_BALL_DISRUPT_PITCH_DEG;

  // Home positions
  constexpr float HAND_REV_HOME                   = BBSM::HAND_REV_HOME;
  constexpr float PITCH_DEG_HOME                  = BBSM::PITCH_DEG_HOME;
  constexpr float YAW_DEG_HOME                    = BBSM::YAW_DEG_HOME;

  // Calibration
  constexpr float CALIB_MAX_YAW_DEG               = BBSM::CALIB_MAX_YAW_DEG;
  constexpr float CALIB_MIN_YAW_DEG               = BBSM::CALIB_MIN_YAW_DEG;
  constexpr float CALIB_YAW_ACCEL                 = BBSM::CALIB_YAW_ACCEL;
  constexpr float CALIB_YAW_DECEL                 = BBSM::CALIB_YAW_DECEL;
  constexpr uint32_t CALIB_PAUSE_MS               = BBSM::CALIB_PAUSE_MS;

  // Retry limits
  constexpr uint8_t MAX_RELOAD_ATTEMPTS           = BBSM::MAX_RELOAD_ATTEMPTS;
  constexpr uint8_t MAX_HOMING_ATTEMPTS           = BBSM::MAX_HOMING_ATTEMPTS;

  // Settle times & thresholds
  constexpr uint32_t PITCH_SETTLE_MS              = BBSM::PITCH_SETTLE_MS;
  constexpr uint32_t PITCH_GRAB_SETTLE_MS         = BBSM::PITCH_GRAB_SETTLE_MS;
  constexpr float    YAW_ANGLE_THRESHOLD_DEG      = BBSM::YAW_ANGLE_THRESHOLD_DEG;

  // Motion timing
  constexpr uint32_t PITCH_IDLE_DELAY_MS          = BBSM::PITCH_IDLE_DELAY_MS;
  constexpr uint32_t MIN_MOTION_DURATION_MS       = BBSM::MIN_MOTION_DURATION_MS;
  constexpr uint32_t PV_FRESHNESS_US              = BBSM::PV_FRESHNESS_US;

  // Axis limits
  constexpr float PITCH_MIN_STOW_DEG              = BBSM::PITCH_MIN_STOW_DEG;
  constexpr float YAW_MIN_ANGLE_DEG               = BBSM::YAW_MIN_ANGLE_DEG;
  constexpr float YAW_MAX_ANGLE_DEG               = BBSM::YAW_MAX_ANGLE_DEG;
}

// ============================================================================
// 11. Ball Detection Configuration — sourced from hardware_config.h (BBBallDetect)
// ============================================================================
namespace BallDetectCfg {
  constexpr uint8_t  GPIO_PIN              = BBBallDetect::GPIO_PIN;
  constexpr uint32_t CHECK_INTERVAL_MS     = BBBallDetect::CHECK_INTERVAL_MS;
  constexpr uint8_t  MAX_MISSING_SAMPLES   = BBBallDetect::MAX_MISSING_SAMPLES;
  constexpr uint32_t CHECK_TIMEOUT_MS      = BBBallDetect::CHECK_TIMEOUT_MS;
}

// ============================================================================
// 12. Operational Constants — sourced from hardware_config.h (BBOp)
// ============================================================================
namespace OpCfg {
  constexpr float    THROW_VEL_MIN_MPS   = BBOp::THROW_VEL_MIN_MPS;
  constexpr float    THROW_VEL_MAX_MPS   = BBOp::THROW_VEL_MAX_MPS;
  constexpr float    SCHEDULE_MARGIN_S   = BBOp::SCHEDULE_MARGIN_S;
  constexpr uint16_t YAW_TELEM_MS        = BBOp::YAW_TELEM_MS;
  constexpr uint32_t SERIAL_BAUD         = BBOp::SERIAL_BAUD;
  constexpr uint32_t SERIAL_WAIT_MS      = BBOp::SERIAL_WAIT_MS;
  constexpr uint32_t ODRIVE_BOOT_MS      = BBOp::ODRIVE_BOOT_MS;
}

// ============================================================================
// 13. Heartbeat Encoding — sourced from hardware_config.h (BBHb)
//     Shared resolutions from protocol_config.h (HeartbeatEncoding).
// ============================================================================
namespace HeartbeatCfg {
  constexpr float YAW_RES_DEG      = HeartbeatEncoding::yaw_res_deg;    // from protocol_config.h
  constexpr float PITCH_RES_DEG    = HeartbeatEncoding::pitch_res_deg;  // from protocol_config.h
  constexpr float HAND_RES_MM      = HeartbeatEncoding::hand_res_mm;    // from protocol_config.h
  constexpr float HAND_MAX_MM      = BBHb::HAND_MAX_MM;
  constexpr float PITCH_CLAMP_MIN  = BBHb::PITCH_CLAMP_MIN_DEG;
  constexpr float PITCH_CLAMP_MAX  = BBHb::PITCH_CLAMP_MAX_DEG;
}
