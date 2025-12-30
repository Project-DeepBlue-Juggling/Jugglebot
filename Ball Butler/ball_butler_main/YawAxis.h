#pragma once
#include <Arduino.h>
#include <SPI.h>

/*
 * =============================================================================
 * YawAxis - Yaw axis controller for brushed DC motor with AS5047P encoder
 * =============================================================================
 * 
 * OVERVIEW:
 *   This class controls a single rotational axis using a brushed DC motor and
 *   an AS5047P absolute magnetic encoder. It runs a PID control loop in an ISR
 *   at a configurable rate (default 150 Hz).
 * 
 * POSITION LIMITS:
 *   - Limits are set via setSoftLimitsDeg(min, max) where min < max.
 *   - Both min and max must be in the range [0, 360).
 *   - The axis will ONLY move through the valid range [min, max].
 *   - Commands outside [min, max] are IGNORED (the axis won't move).
 *   - The axis will NEVER take the "short path" through the forbidden zone,
 *     even if that path would be shorter.
 * 
 * HARD LIMIT PROTECTION:
 *   - If the axis overshoots beyond (min - HARD_LIMIT_OVERSHOOT_DEG_) or
 *     (max + HARD_LIMIT_OVERSHOOT_DEG_), the motor is immediately disabled.
 *   - This protects against mechanical damage from runaway conditions.
 *   - Default overshoot tolerance is 3.0 degrees.
 * 
 * ZERO OFFSET:
 *   - All positions are reported relative to a configurable zero offset.
 *   - Use setZeroHere() to make the current position the new 0°.
 *   - Limits are specified in user coordinates (after zero offset is applied).
 * 
 * USAGE EXAMPLE:
 *   YawAxis yaw(CS_PIN, INA1, INA2, PWM_PIN);
 *   yaw.begin(150.0f);                    // 150 Hz control loop
 *   yaw.setSoftLimitsDeg(10.0f, 190.0f);  // Valid range: 10° to 190°
 *   yaw.setTargetDeg(100.0f);             // Move to 100°
 * 
 * =============================================================================
 */

class YawAxis {
public:
  /* ---------------------------------------------------------------------------
   * Telemetry - Snapshot of axis state for monitoring/debugging
   * ---------------------------------------------------------------------------
   * This structure provides a complete view of the axis state at a single
   * point in time. All values are captured atomically (interrupts disabled).
   */
  struct Telemetry {
    // Motor state
    bool    enabled;          // True if motor is currently being driven
    bool    estop;            // True if e-stop is latched (motor disabled)
    bool    hard_limit_fault; // True if hard limit was exceeded (motor disabled)
    
    // Position and velocity
    float   pos_deg;          // Current position in degrees [0, 360)
    float   vel_rps;          // Current velocity in revolutions per second
    float   cmd_deg;          // Commanded target position in degrees
    float   err_deg;          // Position error in degrees (cmd - pos, via valid path)
    
    // Control signals
    float   u;                // Raw PID output (before slew limiting)
    float   u_slew;           // PID output after slew rate limiting
    int16_t pwm;              // Final PWM command sent to motor (-255 to +255)
    
    // Encoder state
    uint16_t raw;             // Raw 14-bit encoder reading
    
    // Configuration
    float   lim_min_deg;      // Lower soft limit in degrees
    float   lim_max_deg;      // Upper soft limit in degrees
    float   hard_overshoot;   // Hard limit overshoot tolerance in degrees
    float   zero_offset_deg;  // Zero offset in degrees
    float   kp, ki, kd;       // PID gains
    float   accelPPS;         // Acceleration limit (PWM counts per second)
    float   decelPPS;         // Deceleration limit (PWM counts per second)
    
    // Command feedback
    uint32_t rejected_count;  // Total commands rejected (out of limits)
    bool    last_cmd_rejected;// True if most recent command was rejected
  };

  /* ---------------------------------------------------------------------------
   * LimitInfo - Limit configuration for external modules
   * ---------------------------------------------------------------------------
   * Provides limit information so other modules can validate commands before
   * sending them to the axis controller.
   */
  struct LimitInfo {
    float min_deg;            // Lower limit in degrees
    float max_deg;            // Upper limit in degrees
    float range_deg;          // Total valid range (max - min)
    float hard_overshoot_deg; // Hard limit overshoot tolerance
  };

  /* ---------------------------------------------------------------------------
   * ProprioCallback - Callback type for real-time position updates
   * ---------------------------------------------------------------------------
   * This callback is invoked from the ISR every control cycle with fresh
   * position and velocity data. Use this to feed data to other real-time
   * systems (e.g., proprioception, logging).
   * 
   * WARNING: This runs in ISR context. Keep the callback fast and don't
   * call any blocking functions.
   * 
   * Parameters:
   *   pos_deg      - Current position in degrees
   *   vel_rps      - Current velocity in revolutions per second
   *   timestamp_us - Timestamp in microseconds (64-bit, handles rollover)
   */
  typedef void (*ProprioCallback)(float pos_deg, float vel_rps, uint64_t timestamp_us);

  /* ---------------------------------------------------------------------------
   * Constructor
   * ---------------------------------------------------------------------------
   * Creates a YawAxis instance with the specified hardware configuration.
   * Does NOT initialize hardware - call begin() to start the controller.
   * 
   * Parameters:
   *   csPin   - SPI chip select pin for AS5047P encoder
   *   ina1    - Motor driver INA1 pin (direction control)
   *   ina2    - Motor driver INA2 pin (direction control)
   *   pwmPin  - Motor driver PWM pin (speed control)
   *   pwmMin  - Minimum PWM value to overcome motor stiction (default: 60)
   *   pwmHz   - PWM frequency in Hz (default: 20000)
   */
  YawAxis(uint8_t csPin, uint8_t ina1, uint8_t ina2, uint8_t pwmPin,
          uint8_t pwmMin = 60, uint32_t pwmHz = 20000);

  /* ---------------------------------------------------------------------------
   * begin() - Initialize hardware and start control loop
   * ---------------------------------------------------------------------------
   * Initializes SPI, GPIO, PWM, reads the initial encoder position, and starts
   * the control loop timer. The motor starts disabled with command = current
   * position (no initial movement).
   * 
   * Parameters:
   *   loopHz - Control loop frequency in Hz (default: 150)
   */
  void begin(float loopHz = 150.0f);

  /* ---------------------------------------------------------------------------
   * end() - Stop control loop and disable motor
   * ---------------------------------------------------------------------------
   * Stops the control loop timer and sets motor PWM to zero. Call this before
   * destroying the object or when shutting down.
   */
  void end();

  /* ===========================================================================
   * TARGET POSITION COMMANDS
   * ===========================================================================
   * All target commands validate the requested position against the configured
   * soft limits. If the target is outside [min, max], the command is IGNORED
   * and the function returns false. Use isCommandValidDeg() to check before
   * sending if you need to warn the user.
   */

  /* ---------------------------------------------------------------------------
   * setTargetDeg() - Set absolute target position in degrees
   * ---------------------------------------------------------------------------
   * Commands the axis to move to the specified position. The axis will take
   * the path through the valid range [min, max], never through the forbidden
   * zone, even if that would be shorter.
   * 
   * Parameters:
   *   deg - Target position in degrees (must be within soft limits)
   * 
   * Returns:
   *   true  - Command accepted, axis will move to target
   *   false - Command rejected (outside limits), axis unchanged
   */
  bool setTargetDeg(float deg);

  /* ---------------------------------------------------------------------------
   * setTargetRev() - Set absolute target position in revolutions
   * ---------------------------------------------------------------------------
   * Same as setTargetDeg() but accepts position in revolutions (0-1).
   */
  bool setTargetRev(float rev);

  /* ---------------------------------------------------------------------------
   * moveRelDeg() - Move relative to current command position
   * ---------------------------------------------------------------------------
   * Adds the specified offset to the current command position. The resulting
   * target must be within soft limits or the command is rejected.
   * 
   * Parameters:
   *   ddeg - Relative movement in degrees (positive = increase angle)
   * 
   * Returns:
   *   true  - Command accepted
   *   false - Command rejected (result would be outside limits)
   */
  bool moveRelDeg(float ddeg);

  /* ---------------------------------------------------------------------------
   * moveRelRev() - Move relative to current command position in revolutions
   * ---------------------------------------------------------------------------
   * Same as moveRelDeg() but accepts offset in revolutions.
   */
  bool moveRelRev(float drev);

  /* ---------------------------------------------------------------------------
   * isCommandValidDeg() / isCommandValidRev() - Check if command would be accepted
   * ---------------------------------------------------------------------------
   * Use these to validate a command before sending. Useful for providing
   * feedback to users when their requested position is out of range.
   * 
   * Parameters:
   *   deg/rev - Position to validate
   * 
   * Returns:
   *   true  - Position is within soft limits and would be accepted
   *   false - Position is outside soft limits and would be rejected
   */
  bool isCommandValidDeg(float deg) const;
  bool isCommandValidRev(float rev) const;

  /* ===========================================================================
   * EMERGENCY STOP
   * ===========================================================================
   */

  /* ---------------------------------------------------------------------------
   * estop() - Emergency stop (latched)
   * ---------------------------------------------------------------------------
   * Immediately disables the motor and latches the e-stop state. The motor
   * will not drive until clearEstop() is called. Use this for emergency
   * situations or when the robot needs to be safe for human interaction.
   */
  void estop();

  /* ---------------------------------------------------------------------------
   * clearEstop() - Clear emergency stop and resume operation
   * ---------------------------------------------------------------------------
   * Clears the e-stop latch, allowing the motor to drive again. The axis will
   * resume moving toward its commanded position if there is position error.
   */
  void clearEstop();

  /* ---------------------------------------------------------------------------
   * clearHardLimitFault() - Clear hard limit fault and resume operation
   * ---------------------------------------------------------------------------
   * Clears the hard limit fault that occurs when the axis exceeds the hard
   * limit overshoot. Only call this after ensuring the axis is back within
   * valid range and the cause of the overshoot has been addressed.
   */
  void clearHardLimitFault();

  /* ===========================================================================
   * TUNING PARAMETERS
   * ===========================================================================
   */

  /* ---------------------------------------------------------------------------
   * setGains() - Set PID controller gains
   * ---------------------------------------------------------------------------
   * Parameters:
   *   kp - Proportional gain (PWM counts per revolution of error)
   *   ki - Integral gain (PWM counts per revolution-second of error)
   *   kd - Derivative gain (PWM counts per revolution/second of velocity)
   */
  void setGains(float kp, float ki, float kd);

  /* ---------------------------------------------------------------------------
   * setAccel() - Set acceleration/deceleration limits
   * ---------------------------------------------------------------------------
   * Limits the rate of change of the PWM command to prevent jerky motion.
   * 
   * Parameters:
   *   accelPPS - Maximum PWM increase per second (acceleration)
   *   decelPPS - Maximum PWM decrease per second (deceleration)
   */
  void setAccel(float accelPPS, float decelPPS);

  /* ---------------------------------------------------------------------------
   * setFF() - Set friction feedforward
   * ---------------------------------------------------------------------------
   * Adds a constant PWM offset in the direction of motion to overcome static
   * friction. This helps the motor start moving at low error values.
   * 
   * Parameters:
   *   ff_pwm - Feedforward PWM value (applied in direction of error)
   */
  void setFF(float ff_pwm);

  /* ---------------------------------------------------------------------------
   * setToleranceRev() - Set position tolerance for auto-enable
   * ---------------------------------------------------------------------------
   * The motor only drives when position error exceeds this tolerance. This
   * creates a dead-band to prevent hunting/oscillation at the target.
   * 
   * Parameters:
   *   pos_tol_rev - Position tolerance in revolutions (default: 0.0014 = ~0.5°)
   */
  void setToleranceRev(float pos_tol_rev);

  /* ---------------------------------------------------------------------------
   * setDir() - Set encoder and motor direction signs
   * ---------------------------------------------------------------------------
   * Configures the direction mapping between encoder counts and motor drive.
   * Use this to correct for wiring or mechanical orientation.
   * 
   * Parameters:
   *   encDir   - Encoder direction: +1 or -1
   *   motorDir - Motor direction: +1 or -1
   */
  void setDir(int8_t encDir, int8_t motorDir);

  /* ---------------------------------------------------------------------------
   * setDeadzoneMin() - Set minimum PWM for deadzone compensation
   * ---------------------------------------------------------------------------
   * PWM values below this threshold use sigma-delta modulation to create
   * effective low-speed motion by pulsing at the minimum PWM level.
   * 
   * Parameters:
   *   pwmMin - Minimum PWM value (0-255)
   */
  void setDeadzoneMin(uint8_t pwmMin);

  /* ---------------------------------------------------------------------------
   * setHardLimitOvershoot() - Set hard limit overshoot tolerance
   * ---------------------------------------------------------------------------
   * If the axis position exceeds the soft limits by more than this amount,
   * the motor is immediately disabled as a safety measure.
   * 
   * Parameters:
   *   deg - Overshoot tolerance in degrees (default: 3.0)
   */
  void setHardLimitOvershoot(float deg);

  /* ===========================================================================
   * ZERO OFFSET CONFIGURATION
   * ===========================================================================
   */

  /* ---------------------------------------------------------------------------
   * setZeroHere() - Set current position as zero
   * ---------------------------------------------------------------------------
   * Makes the current encoder position the new zero reference. All subsequent
   * position readings and commands will be relative to this point.
   */
  void setZeroHere();

  /* ---------------------------------------------------------------------------
   * setZeroOffset() - Set explicit zero offset
   * ---------------------------------------------------------------------------
   * Sets the zero offset to a specific value in degrees.
   * 
   * Parameters:
   *   offset_deg - Raw encoder position that corresponds to user position 0°
   */
  void setZeroOffset(float offset_deg);

  /* ---------------------------------------------------------------------------
   * getZeroOffset() - Get current zero offset
   * ---------------------------------------------------------------------------
   * Returns:
   *   Current zero offset in degrees
   */
  float getZeroOffset() const;

  /* ---------------------------------------------------------------------------
   * getGains() / getAccel() - Get current tuning parameters
   * ---------------------------------------------------------------------------
   */
  void getGains(float& kp, float& ki, float& kd) const;
  void getAccel(float& accelPPS, float& decelPPS) const;

  /* ===========================================================================
   * SOFT LIMIT CONFIGURATION
   * ===========================================================================
   */

  /* ---------------------------------------------------------------------------
   * setSoftLimitsDeg() - Set position limits in degrees
   * ---------------------------------------------------------------------------
   * Configures the valid position range. Both min and max must be in [0, 360)
   * and min must be less than max. Commands outside this range are rejected,
   * and the axis will never take a path through the forbidden zone.
   * 
   * Parameters:
   *   min_deg - Lower limit in degrees (must be < max_deg)
   *   max_deg - Upper limit in degrees (must be > min_deg)
   */
  void setSoftLimitsDeg(float min_deg, float max_deg);

  /* ---------------------------------------------------------------------------
   * setSoftLimitsRev() - Set position limits in revolutions
   * ---------------------------------------------------------------------------
   * Same as setSoftLimitsDeg() but accepts values in revolutions (0-1).
   */
  void setSoftLimitsRev(float min_rev, float max_rev);

  /* ---------------------------------------------------------------------------
   * getSoftLimitsDeg() - Get current limits in degrees
   * ---------------------------------------------------------------------------
   */
  void getSoftLimitsDeg(float& min_deg, float& max_deg) const;

  /* ---------------------------------------------------------------------------
   * getLimitInfo() - Get complete limit information
   * ---------------------------------------------------------------------------
   * Returns a LimitInfo structure with all limit-related configuration.
   * Useful for external modules that need to validate commands.
   */
  LimitInfo getLimitInfo() const;

  /* ===========================================================================
   * COMMAND FEEDBACK
   * ===========================================================================
   */

  /* ---------------------------------------------------------------------------
   * getAndClearCmdRejected() - Check if last command was rejected
   * ---------------------------------------------------------------------------
   * Returns true if the most recent command was rejected (out of limits).
   * Clears the flag after reading.
   */
  bool getAndClearCmdRejected();

  /* ---------------------------------------------------------------------------
   * getRejectedCount() - Get total rejected command count
   * ---------------------------------------------------------------------------
   * Returns the number of commands rejected since boot. Does not clear.
   */
  uint32_t getRejectedCount() const;

  /* ===========================================================================
   * TELEMETRY AND STATE QUERIES
   * ===========================================================================
   */

  /* ---------------------------------------------------------------------------
   * readTelemetry() - Get complete state snapshot
   * ---------------------------------------------------------------------------
   * Returns a Telemetry structure with all axis state. Values are captured
   * atomically (interrupts disabled during read).
   */
  Telemetry readTelemetry() const;

  /* ---------------------------------------------------------------------------
   * isEnabled() - Check if motor is currently driving
   * ---------------------------------------------------------------------------
   */
  bool isEnabled() const { return enabled_; }

  /* ---------------------------------------------------------------------------
   * isEstopped() - Check if e-stop is latched
   * ---------------------------------------------------------------------------
   */
  bool isEstopped() const { return estop_; }

  /* ---------------------------------------------------------------------------
   * isHardLimitFault() - Check if hard limit fault is active
   * ---------------------------------------------------------------------------
   */
  bool isHardLimitFault() const { return hard_limit_fault_; }

  /* ---------------------------------------------------------------------------
   * getPositionDeg() - Get current position in degrees
   * ---------------------------------------------------------------------------
   * Returns position with zero offset applied. Interrupt-safe.
   */
  float getPositionDeg() const;

  /* ---------------------------------------------------------------------------
   * getVelocityRps() - Get current velocity in rev/s
   * ---------------------------------------------------------------------------
   * Interrupt-safe.
   */
  float getVelocityRps() const;

  /* ===========================================================================
   * PROPRIOCEPTION CALLBACK
   * ===========================================================================
   */

  /* ---------------------------------------------------------------------------
   * setProprioceptionCallback() - Register real-time position callback
   * ---------------------------------------------------------------------------
   * Sets a callback function that will be invoked from the control ISR every
   * cycle with fresh position and velocity data. Pass nullptr to disable.
   * 
   * WARNING: The callback runs in ISR context. Keep it fast!
   */
  void setProprioceptionCallback(ProprioCallback cb);

private:
  // ISR trampoline (static -> instance)
  static void isrTrampoline();
  void controlISR();

  // Hardware interface
  uint16_t readAS();
  void     driveBM_raw(int16_t pwm_abs, int8_t sign);
  void     driveBM(int16_t signed_pwm);
  int16_t  deadzone_compensate(float u);

  // Angle and coordinate transformation helpers
  float normalizeAngle_(float angle_deg) const;
  float rawToEncoderDeg_(uint16_t raw) const;
  float encoderToUserDeg_(float enc_deg) const;
  float userToEncoderDeg_(float user_deg) const;
  bool  isInValidRange_(float pos_deg) const;
  bool  isInHardLimitRange_(float pos_deg) const;

  // 64-bit microsecond timestamp (handles rollover)
  static uint64_t micros64_();

private:
  // Hardware pins
  const uint8_t  CS_PIN_, INA1_PIN_, INA2_PIN_, PWM_PIN_;
  volatile uint8_t PWM_MIN_;
  const uint32_t PWM_HZ_;

  // Control timing
  float LOOP_HZ_;
  float DT_;

  // Constants
  static constexpr uint16_t AS5047_NOP = 0xFFFF;
  static constexpr float    CPR       = 16384.0f;
  static constexpr float    DEG2REV   = 1.0f / 360.0f;
  static constexpr float    REV2DEG   = 360.0f;
  static constexpr uint8_t  PWM_MAX   = 255;

  // PID tuning (volatile for ISR access)
  volatile float Kp_          = 120.0f;
  volatile float Ki_          = 10.0f;
  volatile float Kd_          = 1.5f;
  volatile float FF_PWM_      = 20.0f;
  volatile float POS_TOL_REV_ = 0.0014f;  // ~0.5°
  volatile float ACCEL_PPS_   = 500.0f;
  volatile float DECEL_PPS_   = 500.0f;
  volatile int8_t ENC_DIR_    = -1;
  volatile int8_t MOTOR_DIR_  = +1;

  // Controller state (volatile for ISR access)
  volatile bool    estop_            = false;
  volatile bool    hard_limit_fault_ = false;
  volatile bool    enabled_          = false;
  volatile float   cmd_pos_deg_      = 0.0f;   // Command in user degrees
  volatile float   pos_deg_          = 0.0f;   // Position in user degrees
  volatile float   vel_rps_          = 0.0f;
  volatile float   i_term_           = 0.0f;
  volatile int16_t last_pwm_cmd_     = 0;
  volatile float   last_u_preDZ_     = 0.0f;
  volatile float   last_u_slew_      = 0.0f;
  volatile float   last_err_deg_     = 0.0f;
  volatile float   sd_accum_         = 0.0f;

  // Encoder state
  volatile uint16_t prev_raw_ = 0;

  // Soft limits (in user degrees, min < max always)
  volatile float LIM_MIN_DEG_ = 0.0f;
  volatile float LIM_MAX_DEG_ = 120.0f;

  // Hard limit overshoot tolerance
  volatile float HARD_LIMIT_OVERSHOOT_DEG_ = 3.0f;

  // Zero offset (raw encoder degrees that = user 0°)
  volatile float ZERO_OFFSET_DEG_ = 0.0f;

  // Command rejection tracking
  volatile uint32_t rejected_count_    = 0;
  volatile bool     last_cmd_rejected_ = false;

  // Proprioception callback
  volatile ProprioCallback proprio_cb_ = nullptr;

  // Timer for control loop
  IntervalTimer timer_;

  // Singleton instance pointer
  static YawAxis* instance_;
};