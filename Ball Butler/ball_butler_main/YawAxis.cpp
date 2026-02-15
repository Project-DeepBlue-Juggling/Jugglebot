/*
 * =============================================================================
 * YawAxis.cpp - Implementation (Revised for proper zero offset handling)
 * =============================================================================
 * 
 * COORDINATE SYSTEM DESIGN:
 * 
 * The encoder returns raw angles in [0, 360). The user wants to work in a
 * coordinate system where their limits [0, 185] are safely away from the
 * 0/360 discontinuity.
 * 
 * We use ZERO_OFFSET_DEG_ to define where "user 0°" is in encoder space:
 *   encoder_deg = user_deg + ZERO_OFFSET_DEG_  (mod 360)
 *   user_deg = encoder_deg - ZERO_OFFSET_DEG_  (mod 360, centered on working range)
 * 
 * Example with ZERO_OFFSET_DEG_ = 10:
 *   User 0°   -> Encoder 10°
 *   User 185° -> Encoder 195°
 *   User -5°  -> Encoder 5° (overshoot past user 0°)
 *   
 * The key insight: we DON'T normalize user coordinates to [0, 360).
 * Instead, we keep them in a range centered around the middle of the
 * valid range, allowing negative values for overshoot past 0°.
 * 
 * LIMIT ENFORCEMENT:
 * 
 * Soft limits [LIM_MIN_DEG_, LIM_MAX_DEG_] define the commanded range.
 * Hard limits extend beyond by HARD_LIMIT_OVERSHOOT_DEG_ to allow for
 * overshoot without triggering a fault.
 * 
 * The axis will NEVER move through the forbidden zone (outside hard limits).
 * If commanded to a position that would require crossing the forbidden zone,
 * the command is rejected.
 * 
 * =============================================================================
 */

#include "YawAxis.h"

// Singleton instance pointer
YawAxis* YawAxis::instance_ = nullptr;

/* =============================================================================
 * Constructor
 * =============================================================================
 */
YawAxis::YawAxis(uint8_t csPin, uint8_t ina1, uint8_t ina2, uint8_t pwmPin,
                 uint8_t pwmMin, uint32_t pwmHz)
  : CS_PIN_(csPin), INA1_PIN_(ina1), INA2_PIN_(ina2), PWM_PIN_(pwmPin),
    PWM_MIN_(pwmMin), PWM_HZ_(pwmHz) {}


/* =============================================================================
 * begin() - Initialize hardware and start control loop
 * =============================================================================
 */
void YawAxis::begin(float loopHz) {
  LOOP_HZ_ = loopHz;
  DT_      = 1.0f / LOOP_HZ_;

  // Configure GPIO pins
  pinMode(CS_PIN_,   OUTPUT);
  pinMode(INA1_PIN_, OUTPUT);
  pinMode(INA2_PIN_, OUTPUT);
  pinMode(PWM_PIN_,  OUTPUT);
  digitalWriteFast(CS_PIN_, HIGH);

  // Initialize SPI and PWM
  SPI.begin();
  analogWriteFrequency(PWM_PIN_, PWM_HZ_);
  analogWriteResolution(8);
  driveBM(0);  // Start with motor off

  // Read initial encoder position and convert to user coordinates
  prev_raw_ = readAS();
  float enc_deg = rawToEncoderDeg_(prev_raw_);
  pos_deg_ = encoderToUserDeg_(enc_deg);
  
  // Set command = current position (no initial movement)
  cmd_pos_deg_ = pos_deg_;

  // Reset controller state
  i_term_           = 0.0f;
  sd_accum_         = 0.0f;
  last_u_slew_      = 0.0f;
  last_err_deg_     = 0.0f;
  estop_            = false;
  hard_limit_fault_ = false;
  enabled_          = false;

  // Register singleton and start timer
  instance_ = this;
  timer_.begin(isrTrampoline, (int)(1e6 / LOOP_HZ_));
}


/* =============================================================================
 * end() - Stop control loop and disable motor
 * =============================================================================
 */
void YawAxis::end() {
  timer_.end();
  driveBM(0);
}


/* =============================================================================
 * Coordinate Transformation Functions
 * =============================================================================
 * 
 * These functions handle the conversion between:
 *   - Raw encoder counts (0 to CPR-1)
 *   - Encoder degrees (0 to 360, with ENC_DIR_ applied)
 *   - User degrees (centered on working range, can be negative for overshoot)
 */

/* ---------------------------------------------------------------------------
 * normalizeAngle_() - Normalize angle to [0, 360) range
 * ---------------------------------------------------------------------------
 */
float YawAxis::normalizeAngle_(float angle_deg) const {
  while (angle_deg < 0.0f) angle_deg += 360.0f;
  while (angle_deg >= 360.0f) angle_deg -= 360.0f;
  return angle_deg;
}

/* ---------------------------------------------------------------------------
 * rawToEncoderDeg_() - Convert raw encoder counts to encoder degrees [0, 360)
 * ---------------------------------------------------------------------------
 */
float YawAxis::rawToEncoderDeg_(uint16_t raw) const {
  float deg = (float(raw) / CPR) * REV2DEG * ENC_DIR_;
  return normalizeAngle_(deg);
}

/* ---------------------------------------------------------------------------
 * encoderToUserDeg_() - Convert encoder degrees to user degrees
 * ---------------------------------------------------------------------------
 * User degrees are NOT normalized to [0, 360). Instead, they are centered
 * around the middle of the valid range to allow for overshoot representation.
 * 
 * For example, with ZERO_OFFSET_DEG_ = 10 and limits [0, 185]:
 *   - Center of range is ~92.5°
 *   - Encoder 10° -> User 0°
 *   - Encoder 5° -> User -5° (overshoot past 0°)
 *   - Encoder 195° -> User 185°
 *   - Encoder 200° -> User 190° (overshoot past 185°)
 */
float YawAxis::encoderToUserDeg_(float enc_deg) const {
  // First, get the raw difference
  float user_deg = enc_deg - ZERO_OFFSET_DEG_;
  
  // Now we need to handle the wrap-around. The key is to center the result
  // around the middle of our working range, not around 180°.
  // 
  // Our working range is [LIM_MIN_DEG_ - overshoot, LIM_MAX_DEG_ + overshoot]
  // The center of soft limits is (LIM_MIN_DEG_ + LIM_MAX_DEG_) / 2
  float range_center = (LIM_MIN_DEG_ + LIM_MAX_DEG_) * 0.5f;
  
  // Adjust user_deg to be within ±180° of the range center
  while (user_deg < range_center - 180.0f) user_deg += 360.0f;
  while (user_deg > range_center + 180.0f) user_deg -= 360.0f;
  
  return user_deg;
}

/* ---------------------------------------------------------------------------
 * userToEncoderDeg_() - Convert user degrees to encoder degrees [0, 360)
 * ---------------------------------------------------------------------------
 */
float YawAxis::userToEncoderDeg_(float user_deg) const {
  return normalizeAngle_(user_deg + ZERO_OFFSET_DEG_);
}

/* ---------------------------------------------------------------------------
 * isInValidRange_() - Check if position is within soft limits
 * ---------------------------------------------------------------------------
 */
bool YawAxis::isInValidRange_(float pos_deg) const {
  return (pos_deg >= LIM_MIN_DEG_) && (pos_deg <= LIM_MAX_DEG_);
}

/* ---------------------------------------------------------------------------
 * isInHardLimitRange_() - Check if position is within hard limits
 * ---------------------------------------------------------------------------
 * Hard limits extend beyond soft limits by HARD_LIMIT_OVERSHOOT_DEG_.
 * This allows for overshoot during motion without triggering a fault.
 */
bool YawAxis::isInHardLimitRange_(float pos_deg) const {
  float hard_min = LIM_MIN_DEG_ - HARD_LIMIT_OVERSHOOT_DEG_;
  float hard_max = LIM_MAX_DEG_ + HARD_LIMIT_OVERSHOOT_DEG_;
  return (pos_deg >= hard_min) && (pos_deg <= hard_max);
}




/* =============================================================================
 * Soft Limit Configuration
 * =============================================================================
 */

/* ---------------------------------------------------------------------------
 * setSoftLimitsDeg() - Set position limits in degrees
 * ---------------------------------------------------------------------------
 */
void YawAxis::setSoftLimitsDeg(float min_deg, float max_deg) {
  // Ensure min < max
  if (min_deg > max_deg) {
    float temp = min_deg;
    min_deg = max_deg;
    max_deg = temp;
  }
  
  // Ensure range is valid (not more than ~355° to leave room for forbidden zone)
  if ((max_deg - min_deg) > 355.0f) {
    max_deg = min_deg + 355.0f;
  }
  
  noInterrupts();
  LIM_MIN_DEG_ = min_deg;
  LIM_MAX_DEG_ = max_deg;
  interrupts();
}

/* ---------------------------------------------------------------------------
 * setSoftLimitsRev() - Set position limits in revolutions
 * ---------------------------------------------------------------------------
 */
void YawAxis::setSoftLimitsRev(float min_rev, float max_rev) {
  setSoftLimitsDeg(min_rev * REV2DEG, max_rev * REV2DEG);
}

/* ---------------------------------------------------------------------------
 * getSoftLimitsDeg() - Get current limits
 * ---------------------------------------------------------------------------
 */
void YawAxis::getSoftLimitsDeg(float& min_deg, float& max_deg) const {
  noInterrupts();
  min_deg = LIM_MIN_DEG_;
  max_deg = LIM_MAX_DEG_;
  interrupts();
}

/* ---------------------------------------------------------------------------
 * getLimitInfo() - Get complete limit information
 * ---------------------------------------------------------------------------
 */
YawAxis::LimitInfo YawAxis::getLimitInfo() const {
  LimitInfo info;
  noInterrupts();
  info.min_deg = LIM_MIN_DEG_;
  info.max_deg = LIM_MAX_DEG_;
  info.range_deg = LIM_MAX_DEG_ - LIM_MIN_DEG_;
  info.hard_overshoot_deg = HARD_LIMIT_OVERSHOOT_DEG_;
  interrupts();
  return info;
}

/* ---------------------------------------------------------------------------
 * setHardLimitOvershoot() - Set hard limit overshoot tolerance
 * ---------------------------------------------------------------------------
 */
void YawAxis::setHardLimitOvershoot(float deg) {
  noInterrupts();
  HARD_LIMIT_OVERSHOOT_DEG_ = fabsf(deg);
  interrupts();
}


/* =============================================================================
 * Command Validation and Execution
 * =============================================================================
 */

/* ---------------------------------------------------------------------------
 * isCommandValidDeg() - Check if command is within limits
 * ---------------------------------------------------------------------------
 */
bool YawAxis::isCommandValidDeg(float deg) const {
  noInterrupts();
  bool valid = (deg >= LIM_MIN_DEG_) && (deg <= LIM_MAX_DEG_);
  interrupts();
  return valid;
}

/* ---------------------------------------------------------------------------
 * isCommandValidRev() - Check if command is within limits (revolutions)
 * ---------------------------------------------------------------------------
 */
bool YawAxis::isCommandValidRev(float rev) const {
  return isCommandValidDeg(rev * REV2DEG);
}

/* ---------------------------------------------------------------------------
 * setTargetDeg() - Set absolute target position
 * ---------------------------------------------------------------------------
 * Returns true if command accepted, false if rejected (outside limits).
 */
bool YawAxis::setTargetDeg(float deg) {
  noInterrupts();
  
  // Validate: must be within soft limits
  if (deg < LIM_MIN_DEG_ || deg > LIM_MAX_DEG_) {
    // Command is outside limits - REJECT
    last_cmd_rejected_ = true;
    rejected_count_++;
    interrupts();
    return false;
  }
  
  // Command is valid - accept it
  cmd_pos_deg_ = deg;
  last_cmd_rejected_ = false;
  interrupts();
  return true;
}

/* ---------------------------------------------------------------------------
 * setTargetRev() - Set absolute target position (revolutions)
 * ---------------------------------------------------------------------------
 */
bool YawAxis::setTargetRev(float rev) {
  return setTargetDeg(rev * REV2DEG);
}

/* ---------------------------------------------------------------------------
 * moveRelDeg() - Move relative to current command
 * ---------------------------------------------------------------------------
 */
bool YawAxis::moveRelDeg(float ddeg) {
  noInterrupts();
  
  float new_target = cmd_pos_deg_ + ddeg;
  
  // Validate: must be within soft limits
  if (new_target < LIM_MIN_DEG_ || new_target > LIM_MAX_DEG_) {
    // Result would be outside limits - REJECT
    last_cmd_rejected_ = true;
    rejected_count_++;
    interrupts();
    return false;
  }
  
  // Command is valid - accept it
  cmd_pos_deg_ = new_target;
  last_cmd_rejected_ = false;
  interrupts();
  return true;
}

/* ---------------------------------------------------------------------------
 * moveRelRev() - Move relative to current command (revolutions)
 * ---------------------------------------------------------------------------
 */
bool YawAxis::moveRelRev(float drev) {
  return moveRelDeg(drev * REV2DEG);
}

/* ---------------------------------------------------------------------------
 * getAndClearCmdRejected() - Check if last command was rejected
 * ---------------------------------------------------------------------------
 */
bool YawAxis::getAndClearCmdRejected() {
  noInterrupts();
  bool r = last_cmd_rejected_;
  last_cmd_rejected_ = false;
  interrupts();
  return r;
}

/* ---------------------------------------------------------------------------
 * getRejectedCount() - Get total rejected commands
 * ---------------------------------------------------------------------------
 */
uint32_t YawAxis::getRejectedCount() const {
  noInterrupts();
  uint32_t count = rejected_count_;
  interrupts();
  return count;
}


/* =============================================================================
 * Emergency Stop and Fault Handling
 * =============================================================================
 */

/* ---------------------------------------------------------------------------
 * estop() - Emergency stop
 * ---------------------------------------------------------------------------
 */
void YawAxis::estop() {
  estop_ = true;
}

/* ---------------------------------------------------------------------------
 * clearEstop() - Clear e-stop and resume normal operation
 * ---------------------------------------------------------------------------
 */
void YawAxis::clearEstop() {
  noInterrupts();
  estop_ = false;
  // Also update command to current position to prevent sudden movement
  cmd_pos_deg_ = pos_deg_;
  interrupts();
}

/* ---------------------------------------------------------------------------
 * clearHardLimitFault() - Clear hard limit fault
 * ---------------------------------------------------------------------------
 * NOTE: Only call this after physically moving the axis back within limits!
 */
void YawAxis::clearHardLimitFault() {
  noInterrupts();
  hard_limit_fault_ = false;
  // Update command to current position
  cmd_pos_deg_ = pos_deg_;
  interrupts();
}


/* =============================================================================
 * Tuning Parameters
 * =============================================================================
 */

void YawAxis::setGains(float kp, float ki, float kd) {
  noInterrupts();
  Kp_ = kp;
  Ki_ = ki;
  Kd_ = kd;
  interrupts();
}

void YawAxis::setAccel(float accelPPS, float decelPPS) {
  noInterrupts();
  ACCEL_PPS_ = fabsf(accelPPS);
  DECEL_PPS_ = fabsf(decelPPS);
  interrupts();
}

void YawAxis::setVelFilterAlpha(float alpha) {
  noInterrupts();
  // Clamp to valid range [0.0, 1.0]
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  VEL_LPF_ALPHA_ = alpha;
  interrupts();
}

void YawAxis::setMaxValidVelRps(float max_vel_rps) {
  noInterrupts();
  // Ensure positive and reasonable
  if (max_vel_rps < 0.1f) max_vel_rps = 0.1f;
  MAX_VALID_VEL_RPS_ = max_vel_rps;
  interrupts();
}

void YawAxis::getGains(float& kp, float& ki, float& kd) const {
  noInterrupts();
  kp = Kp_;
  ki = Ki_;
  kd = Kd_;
  interrupts();
}

void YawAxis::getAccel(float& accelPPS, float& decelPPS) const {
  noInterrupts();
  accelPPS = ACCEL_PPS_;
  decelPPS = DECEL_PPS_;
  interrupts();
}

void YawAxis::setFF(float ff_pwm) {
  noInterrupts();
  FF_PWM_ = ff_pwm;
  interrupts();
}

void YawAxis::setToleranceRev(float pos_tol_rev) {
  noInterrupts();
  POS_TOL_REV_ = fabsf(pos_tol_rev);
  interrupts();
}

void YawAxis::setDir(int8_t encDir, int8_t motorDir) {
  noInterrupts();
  ENC_DIR_   = (encDir >= 0)   ? +1 : -1;
  MOTOR_DIR_ = (motorDir >= 0) ? +1 : -1;
  interrupts();
}

void YawAxis::setDeadzoneMin(uint8_t pwmMin) {
  noInterrupts();
  PWM_MIN_ = pwmMin;
  interrupts();
}


/* =============================================================================
 * Zero Offset Configuration
 * =============================================================================
 */

/* ---------------------------------------------------------------------------
 * setZeroHere() - Set current position as zero
 * ---------------------------------------------------------------------------
 * Makes the current physical position become user 0°.
 */
void YawAxis::setZeroHere() {
  noInterrupts();
  
  // Get current encoder position
  float enc_deg = rawToEncoderDeg_(prev_raw_);
  
  // This encoder position should map to user 0°
  // user_deg = enc_deg - offset  =>  0 = enc_deg - offset  =>  offset = enc_deg
  ZERO_OFFSET_DEG_ = enc_deg;
  
  // Update positions to reflect new coordinate system
  pos_deg_ = 0.0f;
  cmd_pos_deg_ = 0.0f;
  
  interrupts();
}

/* ---------------------------------------------------------------------------
 * setZeroOffset() - Set explicit zero offset
 * ---------------------------------------------------------------------------
 * Sets ZERO_OFFSET_DEG_ directly. This is the encoder angle that corresponds
 * to user 0°.
 * 
 * IMPORTANT: When changing the offset, both position and command must be
 * transformed to maintain the same physical targets.
 */
void YawAxis::setZeroOffset(float offset_deg) {
  noInterrupts();
  
  // Get current encoder position (this doesn't change)
  float enc_deg = rawToEncoderDeg_(prev_raw_);
  
  // Calculate what the current command is in encoder space (using OLD offset)
  float cmd_enc_deg = userToEncoderDeg_(cmd_pos_deg_);
  
  // Update offset
  ZERO_OFFSET_DEG_ = normalizeAngle_(offset_deg);
  
  // Recalculate user coordinates with new offset
  pos_deg_ = encoderToUserDeg_(enc_deg);
  cmd_pos_deg_ = encoderToUserDeg_(cmd_enc_deg);
  
  interrupts();
}

/* ---------------------------------------------------------------------------
 * getZeroOffset() - Get current zero offset
 * ---------------------------------------------------------------------------
 */
float YawAxis::getZeroOffset() const {
  noInterrupts();
  float offset = ZERO_OFFSET_DEG_;
  interrupts();
  return offset;
}


/* =============================================================================
 * Telemetry
 * =============================================================================
 */

YawAxis::Telemetry YawAxis::readTelemetry() const {
  Telemetry t;
  
  noInterrupts();
  t.enabled          = enabled_;
  t.estop            = estop_;
  t.hard_limit_fault = hard_limit_fault_;
  t.pos_deg          = pos_deg_;
  t.vel_rps          = vel_rps_;
  t.vel_rps_raw      = vel_rps_raw_;
  t.cmd_deg          = cmd_pos_deg_;
  t.err_deg          = last_err_deg_;
  t.u                = last_u_preDZ_;
  t.u_slew           = last_u_slew_;
  t.pwm              = last_pwm_cmd_;
  t.raw              = prev_raw_;
  t.lim_min_deg      = LIM_MIN_DEG_;
  t.lim_max_deg      = LIM_MAX_DEG_;
  t.hard_overshoot   = HARD_LIMIT_OVERSHOOT_DEG_;
  t.zero_offset_deg  = ZERO_OFFSET_DEG_;
  t.kp               = Kp_;
  t.ki               = Ki_;
  t.kd               = Kd_;
  t.accelPPS         = ACCEL_PPS_;
  t.decelPPS         = DECEL_PPS_;
  t.vel_lpf_alpha    = VEL_LPF_ALPHA_;
  t.max_valid_vel_rps = MAX_VALID_VEL_RPS_;
  t.glitch_count     = glitch_count_;
  t.rejected_count   = rejected_count_;
  t.last_cmd_rejected = last_cmd_rejected_;
  interrupts();
  
  return t;
}

float YawAxis::getPositionDeg() const {
  noInterrupts();
  float p = pos_deg_;
  interrupts();
  return p;
}

float YawAxis::getVelocityRps() const {
  noInterrupts();
  float v = vel_rps_;
  interrupts();
  return v;
}


/* =============================================================================
 * Proprioception Callback
 * =============================================================================
 */

void YawAxis::setProprioceptionCallback(ProprioCallback cb) {
  noInterrupts();
  proprio_cb_ = cb;
  interrupts();
}


/* =============================================================================
 * ISR Trampoline
 * =============================================================================
 */

void YawAxis::isrTrampoline() {
  if (instance_) instance_->controlISR();
}


/* =============================================================================
 * Hardware Interface
 * =============================================================================
 */

/* ---------------------------------------------------------------------------
 * readAS() - Read raw value from AS5047P encoder
 * ---------------------------------------------------------------------------
 */
uint16_t YawAxis::readAS() {
  SPI.beginTransaction(SPISettings(1'000'000, MSBFIRST, SPI_MODE1));
  digitalWriteFast(CS_PIN_, LOW);
  uint16_t rx = SPI.transfer16(AS5047_NOP);
  digitalWriteFast(CS_PIN_, HIGH);
  SPI.endTransaction();
  return rx & 0x3FFF;  // 14-bit value
}

/* ---------------------------------------------------------------------------
 * driveBM_raw() - Raw motor drive (PWM + direction)
 * ---------------------------------------------------------------------------
 */
void YawAxis::driveBM_raw(int16_t pwm_abs, int8_t sign) {
  if (sign > 0) {
    digitalWriteFast(INA1_PIN_, HIGH);
    digitalWriteFast(INA2_PIN_, LOW);
  } else if (sign < 0) {
    digitalWriteFast(INA1_PIN_, LOW);
    digitalWriteFast(INA2_PIN_, HIGH);
  } else {
    digitalWriteFast(INA1_PIN_, LOW);
    digitalWriteFast(INA2_PIN_, LOW);
  }
  analogWrite(PWM_PIN_, pwm_abs);
}

/* ---------------------------------------------------------------------------
 * driveBM() - Drive motor with signed PWM value
 * ---------------------------------------------------------------------------
 */
void YawAxis::driveBM(int16_t signed_pwm) {
  int8_t sign = 0;
  int16_t mag = signed_pwm;
  if (mag > 0) {
    sign = +1;
  } else if (mag < 0) {
    sign = -1;
    mag = -mag;
  }
  sign *= MOTOR_DIR_;
  driveBM_raw(mag, sign);
}

/* ---------------------------------------------------------------------------
 * deadzone_compensate() - Sigma-delta modulation for low PWM
 * ---------------------------------------------------------------------------
 */
int16_t YawAxis::deadzone_compensate(float u) {
  if (u == 0.0f) {
    sd_accum_ = 0.0f;
    return 0;
  }
  
  float mag = fabsf(u);
  int8_t sgn = (u > 0) ? 1 : -1;
  
  if (mag >= PWM_MIN_) {
    // Above deadzone - output directly
    sd_accum_ = 0.0f;
    mag = fminf(mag, (float)PWM_MAX);
    return sgn * (int16_t)roundf(mag);
  }
  
  // Below deadzone - use sigma-delta modulation
  sd_accum_ += mag;
  if (sd_accum_ >= PWM_MIN_) {
    sd_accum_ -= PWM_MIN_;
    return sgn * PWM_MIN_;
  }
  
  return 0;
}


/* =============================================================================
 * Control ISR - Main Control Loop
 * =============================================================================
 * 
 * This runs at LOOP_HZ_ (default 150 Hz) and implements:
 *   1. Encoder reading and position tracking
 *   2. Position error calculation
 *   3. Hard limit checking
 *   4. PID control with anti-windup
 *   5. Boundary enforcement (prevent motion toward forbidden zone)
 *   6. Slew rate limiting
 *   7. Deadzone compensation
 *   8. Proprioception callback
 */
void YawAxis::controlISR() {
  
  // =========================================================================
  // STEP 1: Read encoder and calculate position
  // =========================================================================
  uint16_t raw = readAS();
  prev_raw_ = raw;
  
  // Convert to encoder degrees, then to user degrees
  float enc_deg = rawToEncoderDeg_(raw);
  float new_pos_deg = encoderToUserDeg_(enc_deg);
  
  // Calculate velocity
  float last_pos = pos_deg_;
  
  // Velocity calculation - simple difference (user coords are not wrapped)
  float delta_pos = new_pos_deg - last_pos;
  
  // Sanity check: if delta is huge, it's probably a glitch or wrap issue
  if (fabsf(delta_pos) > 180.0f) {
    // Likely a coordinate wrap - don't count this as real velocity
    delta_pos = 0.0f;
  }
  
  // GLITCH REJECTION: If the implied velocity is impossibly high,
  // reject this reading as electrical noise and keep the previous position
  float implied_vel_rps = fabsf(delta_pos * DEG2REV) / DT_;
  if (implied_vel_rps > MAX_VALID_VEL_RPS_) {
    // This is likely a noise glitch - reject the reading
    glitch_count_++;
    new_pos_deg = last_pos;  // Keep previous position
    delta_pos = 0.0f;        // Zero velocity
  }
  
  pos_deg_ = new_pos_deg;
  vel_rps_raw_ = (delta_pos * DEG2REV) / DT_;
  
  // Low-pass filter on velocity to reduce noise-induced jitter
  // EMA: vel_filtered = alpha * vel_raw + (1 - alpha) * vel_filtered_prev
  vel_rps_ = VEL_LPF_ALPHA_ * vel_rps_raw_ + (1.0f - VEL_LPF_ALPHA_) * vel_rps_;
  
  // =========================================================================
  // STEP 2: Check hard limits (safety first!)
  // =========================================================================
  if (!isInHardLimitRange_(pos_deg_)) {
    // Position exceeded hard limits - FAULT!
    hard_limit_fault_ = true;
    driveBM(0);
    enabled_ = false;
    i_term_ = 0.0f;
    sd_accum_ = 0.0f;
    last_pwm_cmd_ = 0;
    last_u_preDZ_ = 0.0f;
    last_u_slew_ = 0.0f;
    last_err_deg_ = 0.0f;
    
    // Fire callback and return
    if (proprio_cb_) {
      proprio_cb_(pos_deg_, vel_rps_, micros64());
    }
    return;
  }
  
  // =========================================================================
  // STEP 3: Calculate position error
  // =========================================================================
  // Error is simply (command - position). Since we're using user coordinates
  // that aren't wrapped to [0, 360), this gives the correct direct path.
  float err_deg = cmd_pos_deg_ - pos_deg_;
  last_err_deg_ = err_deg;
  
  float err_rev = err_deg * DEG2REV;
  
  // =========================================================================
  // STEP 4: Check if motor should be enabled
  // =========================================================================
  bool should_enable = (!estop_) && 
                       (!hard_limit_fault_) && 
                       (fabsf(err_rev) > POS_TOL_REV_);
  
  if (!should_enable) {
    // Disable motor and reset integrator
    driveBM(0);
    enabled_ = false;
    i_term_ = 0.0f;
    sd_accum_ = 0.0f;
    last_pwm_cmd_ = 0;
    last_u_preDZ_ = 0.0f;
    last_u_slew_ = 0.0f;
    
    // Fire callback and return
    if (proprio_cb_) {
      proprio_cb_(pos_deg_, vel_rps_, micros64());
    }
    return;
  }
  
  enabled_ = true;
  
  // =========================================================================
  // STEP 5: PID Control
  // =========================================================================
  
  // Proportional term
  float p = Kp_ * err_rev;
  
  // Derivative term (on velocity, not error, to avoid setpoint kick)
  float d = -Kd_ * vel_rps_;
  
  // Feedforward for friction
  float u_ff = (err_rev > 0) ? FF_PWM_ : ((err_rev < 0) ? -FF_PWM_ : 0.0f);
  
  // Total control effort (before limiting)
  float u = p + d + u_ff + i_term_;
  last_u_preDZ_ = u;
  
  // -------------------------------------------------------------------------
  // Anti-windup: only accumulate integral if not saturated
  // -------------------------------------------------------------------------
  float u_limited = fmaxf(fminf(u, (float)PWM_MAX), -(float)PWM_MAX);
  if (fabsf(u_limited) < 0.95f * PWM_MAX) {
    i_term_ += Ki_ * err_rev * DT_;
    i_term_ = fmaxf(fminf(i_term_, (float)PWM_MAX), -(float)PWM_MAX);
  }
  
  // =========================================================================
  // STEP 6: Boundary enforcement
  // =========================================================================
  // At the soft limit boundaries, prevent motion that would exit the valid range.
  // 
  // In user coordinates:
  //   - Positive control effort (u > 0) moves toward higher user degrees
  //   - Negative control effort (u < 0) moves toward lower user degrees
  //
  // We allow motion INTO the valid range even if slightly outside,
  // but prevent motion OUT OF the valid range.
  
  float u_target = u_limited;
  const float BOUNDARY_TOL = 0.5f;  // degrees
  
  // Near or beyond max limit: don't allow positive motion (would increase pos_deg_)
  if (pos_deg_ >= LIM_MAX_DEG_ - BOUNDARY_TOL && u_target > 0) {
    u_target = 0;
    i_term_ = 0.0f;  // Reset integrator to prevent windup at boundary
  }
  
  // Near or beyond min limit: don't allow negative motion (would decrease pos_deg_)
  if (pos_deg_ <= LIM_MIN_DEG_ + BOUNDARY_TOL && u_target < 0) {
    u_target = 0;
    i_term_ = 0.0f;  // Reset integrator to prevent windup at boundary
  }
  
  // =========================================================================
  // STEP 7: Slew rate limiting (acceleration/deceleration)
  // =========================================================================
  // Determine if we're accelerating or decelerating based on MAGNITUDE change,
  // not signed value. This ensures symmetric behavior for CW vs CCW motion.
  // - Accelerating: |u_target| > |last_u_slew_| (magnitude increasing)
  // - Decelerating: |u_target| < |last_u_slew_| (magnitude decreasing)
  float mag_target = fabsf(u_target);
  float mag_last   = fabsf(last_u_slew_);
  bool is_accelerating = (mag_target > mag_last);
  
  float step = (is_accelerating ? ACCEL_PPS_ : DECEL_PPS_) * DT_;
  float du = u_target - last_u_slew_;
  
  if (du > 0) {
    last_u_slew_ += fminf(du, step);
  } else if (du < 0) {
    last_u_slew_ -= fminf(-du, step);
  }
  
  if (fabsf(last_u_slew_) < 1e-3f) {
    last_u_slew_ = 0.0f;
  }
  
  // =========================================================================
  // STEP 8: Deadzone compensation and output
  // =========================================================================
  int16_t pwm_cmd = deadzone_compensate(last_u_slew_);
  
  // Final boundary check on PWM output
  if (pos_deg_ >= LIM_MAX_DEG_ - BOUNDARY_TOL && pwm_cmd > 0) {
    pwm_cmd = 0;
  }
  if (pos_deg_ <= LIM_MIN_DEG_ + BOUNDARY_TOL && pwm_cmd < 0) {
    pwm_cmd = 0;
  }
  
  last_pwm_cmd_ = pwm_cmd;
  driveBM(pwm_cmd);

  // =========================================================================
  // STEP 9: Fire proprioception callback
  // =========================================================================
  if (proprio_cb_) {
    proprio_cb_(pos_deg_, vel_rps_, micros64());
  }
}