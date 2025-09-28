#include "YawAxis.h"

YawAxis* YawAxis::instance_ = nullptr;

YawAxis::YawAxis(uint8_t csPin, uint8_t ina1, uint8_t ina2, uint8_t pwmPin,
                 uint8_t pwmMin, uint32_t pwmHz)
: CS_PIN_(csPin), INA1_PIN_(ina1), INA2_PIN_(ina2), PWM_PIN_(pwmPin),
  PWM_MIN_(pwmMin), PWM_HZ_(pwmHz) {}

void YawAxis::begin(float loopHz){
  LOOP_HZ_ = loopHz;
  DT_      = 1.0f / LOOP_HZ_;

  pinMode(CS_PIN_,   OUTPUT);
  pinMode(INA1_PIN_, OUTPUT);
  pinMode(INA2_PIN_, OUTPUT);
  pinMode(PWM_PIN_,  OUTPUT);
  digitalWriteFast(CS_PIN_, HIGH);

  SPI.begin();
  analogWriteFrequency(PWM_PIN_, PWM_HZ_);
  analogWriteResolution(8);
  driveBM(0);

  prev_raw_    = readAS();
  turn_count_  = 0;

  // Absolute position from encoder; zero is whatever the encoder defines as zero.
  float abs_rev = float(prev_raw_) / CPR;
  pos_rev_      = ENC_DIR_ * abs_rev;

  // IMPORTANT: Command = current position so there is no error => no motion on boot.
  cmd_pos_rev_  = pos_rev_;

  i_term_       = 0.0f;
  sd_accum_     = 0.0f;
  last_u_slew_  = 0.0f;
  estop_        = false;
  enabled_      = false;

  instance_ = this;
  timer_.begin(isrTrampoline, (int)(1e6 / LOOP_HZ_));
}

void YawAxis::end(){
  timer_.end();
  driveBM(0);
}

/* -------------------- Limits config -------------------- */
void YawAxis::setSoftLimitsDeg(float min_deg, float max_deg){
  setSoftLimitsRev(min_deg * DEG2REV, max_deg * DEG2REV);
}
void YawAxis::setSoftLimitsRev(float min_rev, float max_rev){
  noInterrupts();
  if (max_rev < min_rev) { float t = min_rev; min_rev = max_rev; max_rev = t; }
  LIM_MIN_REV_ = min_rev;
  LIM_MAX_REV_ = max_rev;
  interrupts();
}

/* --------------- Command setters (with clamp) --------- */
float YawAxis::clampToLimitsRev_(float rev, bool& hitLow, bool& hitHigh){
  hitLow = hitHigh = false;
  if (rev < LIM_MIN_REV_) { hitLow = true;  return LIM_MIN_REV_; }
  if (rev > LIM_MAX_REV_) { hitHigh = true; return LIM_MAX_REV_; }
  return rev;
}

void YawAxis::setTargetDeg(float deg){
  bool lo=false, hi=false;
  float rev = deg * DEG2REV;
  float clamped = clampToLimitsRev_(rev, lo, hi);
  noInterrupts();
  cmd_pos_rev_ = clamped;
  if (lo || hi) { last_cmd_clipped_ = true; last_clip_low_ = lo; last_clip_high_ = hi; clip_count_++; }
  interrupts();
}
void YawAxis::setTargetRev(float rev){
  bool lo=false, hi=false;
  float clamped = clampToLimitsRev_(rev, lo, hi);
  noInterrupts();
  cmd_pos_rev_ = clamped;
  if (lo || hi) { last_cmd_clipped_ = true; last_clip_low_ = lo; last_clip_high_ = hi; clip_count_++; }
  interrupts();
}
void YawAxis::moveRelDeg(float ddeg){
  bool lo=false, hi=false;
  noInterrupts();
  float wanted = cmd_pos_rev_ + ddeg * DEG2REV;
  float clamped = clampToLimitsRev_(wanted, lo, hi);
  cmd_pos_rev_ = clamped;
  if (lo || hi) { last_cmd_clipped_ = true; last_clip_low_ = lo; last_clip_high_ = hi; clip_count_++; }
  interrupts();
}
void YawAxis::moveRelRev(float drev){
  bool lo=false, hi=false;
  noInterrupts();
  float wanted = cmd_pos_rev_ + drev;
  float clamped = clampToLimitsRev_(wanted, lo, hi);
  cmd_pos_rev_ = clamped;
  if (lo || hi) { last_cmd_clipped_ = true; last_clip_low_ = lo; last_clip_high_ = hi; clip_count_++; }
  interrupts();
}

bool YawAxis::getAndClearCmdClipped(bool* clippedLow, bool* clippedHigh){
  bool r=false, lo=false, hi=false;
  noInterrupts();
  r  = last_cmd_clipped_;
  lo = last_clip_low_;
  hi = last_clip_high_;
  last_cmd_clipped_ = false;
  last_clip_low_    = false;
  last_clip_high_   = false;
  interrupts();
  if (clippedLow)  *clippedLow  = lo;
  if (clippedHigh) *clippedHigh = hi;
  return r;
}

/* -------------------- Convenience ---------------------- */
void YawAxis::estop(){ estop_ = true; driveBM(0); }
void YawAxis::clearEstop(){ estop_ = false; }

void YawAxis::setGains(float kp, float ki, float kd){ Kp_ = kp; Ki_ = ki; Kd_ = kd; }
void YawAxis::setAccel(float accelPPS, float decelPPS){
  ACCEL_PPS_ = fabsf(accelPPS);
  DECEL_PPS_ = fabsf(decelPPS);
}
void YawAxis::setFF(float ff_pwm){ FF_PWM_ = ff_pwm; }
void YawAxis::setToleranceRev(float pos_tol_rev){ POS_TOL_REV_ = fabsf(pos_tol_rev); }
void YawAxis::setDir(int8_t encDir, int8_t motorDir){ ENC_DIR_ = (encDir>=0)?+1:-1; MOTOR_DIR_ = (motorDir>=0)?+1:-1; }
void YawAxis::setDeadzoneMin(uint8_t pwmMin){ PWM_MIN_ = pwmMin; }

/* -------------------- Telemetry ------------------------ */
YawAxis::Telemetry YawAxis::readTelemetry() const {
  Telemetry t;
  noInterrupts();
  t.enabled   = enabled_;
  t.estop     = estop_;
  t.pos_deg   = pos_rev_ * REV2DEG;
  t.vel_rps   = vel_rps_;
  t.cmd_deg   = cmd_pos_rev_ * REV2DEG;
  t.err_deg   = (cmd_pos_rev_ - pos_rev_) * REV2DEG;
  t.u         = last_u_preDZ_;
  t.u_slew    = last_u_slew_;
  t.pwm       = last_pwm_cmd_;
  t.turn      = turn_count_;
  t.raw       = prev_raw_;
  t.accelPPS  = ACCEL_PPS_;
  t.decelPPS  = DECEL_PPS_;
  t.lim_min_deg = LIM_MIN_REV_ * REV2DEG;
  t.lim_max_deg = LIM_MAX_REV_ * REV2DEG;
  t.at_min    = (pos_rev_ <= LIM_MIN_REV_);
  t.at_max    = (pos_rev_ >= LIM_MAX_REV_);
  t.clip_count = clip_count_;
  t.last_cmd_clipped = last_cmd_clipped_;
  interrupts();
  return t;
}

/* ======== private ======== */
void YawAxis::isrTrampoline(){
  if (instance_) instance_->controlISR();
}

uint16_t YawAxis::readAS() {
  uint16_t a;
  SPI.beginTransaction(SPISettings(1'000'000, MSBFIRST, SPI_MODE1));
  digitalWriteFast(CS_PIN_, LOW);  SPI.transfer16(AS5047_NOP);     digitalWriteFast(CS_PIN_, HIGH);
  digitalWriteFast(CS_PIN_, LOW);  a = SPI.transfer16(AS5047_NOP); digitalWriteFast(CS_PIN_, HIGH);
  SPI.endTransaction();
  return a & 0x3FFF;
}

void YawAxis::driveBM_raw(int16_t pwm_abs, int8_t sign){
  if (sign > 0)      { digitalWriteFast(INA1_PIN_, HIGH); digitalWriteFast(INA2_PIN_, LOW ); }
  else if (sign < 0) { digitalWriteFast(INA1_PIN_, LOW ); digitalWriteFast(INA2_PIN_, HIGH); }
  else               { digitalWriteFast(INA1_PIN_, LOW ); digitalWriteFast(INA2_PIN_, LOW ); }
  analogWrite(PWM_PIN_, constrain(pwm_abs, 0, PWM_MAX));
}

void YawAxis::driveBM(int16_t signed_pwm){
  int8_t sign = 0; int16_t mag = signed_pwm;
  if (mag > 0)      { sign = +1; }
  else if (mag < 0) { sign = -1; mag = -mag; }
  sign *= MOTOR_DIR_;
  driveBM_raw(mag, sign);
}

int16_t YawAxis::deadzone_compensate(float u){
  if (u == 0.0f) { sd_accum_ = 0.0f; return 0; }
  float mag = fabsf(u);
  int8_t sgn = (u > 0) ? 1 : -1;
  if (mag >= PWM_MIN_) {
    sd_accum_ = 0.0f;
    mag = fminf(mag, (float)PWM_MAX);
    return sgn * (int16_t)roundf(mag);
  }
  sd_accum_ += mag;
  if (sd_accum_ >= PWM_MIN_) {
    sd_accum_ -= PWM_MIN_;
    return sgn * PWM_MIN_;
  }
  return 0;
}

void YawAxis::controlISR(){
  // 1) Read/unwrap encoder
  uint16_t raw = readAS();
  int16_t diff = (int16_t)(raw - prev_raw_);
  if      (diff >  8192) turn_count_--;          // wrap 0->max (reverse)
  else if (diff < -8192) turn_count_++;          // wrap max->0 (forward)
  prev_raw_ = raw;

  float abs_rev   = (float)raw / CPR;
  float multi_rev = (float)turn_count_ + abs_rev;
  float last_pos  = pos_rev_;
  pos_rev_ = ENC_DIR_ * multi_rev;
  vel_rps_ = (pos_rev_ - last_pos) / DT_;

  // 2) Auto gating
  float err = cmd_pos_rev_ - pos_rev_;
  last_err_ = err;

  bool auto_enable = (!estop_) && (fabsf(err) > POS_TOL_REV_);
  if (!auto_enable) {
    driveBM(0);
    enabled_      = false;
    i_term_       = 0.0f;
    sd_accum_     = 0.0f;
    last_pwm_cmd_ = 0;
    last_u_preDZ_ = 0.0f;
    last_u_slew_  = 0.0f;
    return;
  }
  enabled_ = true;

  // 3) PID (D on measurement)
  float p = Kp_ * err;
  float d = -Kd_ * vel_rps_;
  float u_ff = (err > 0 ? FF_PWM_ : (err < 0 ? -FF_PWM_ : 0.0f));
  float u = p + d + u_ff + i_term_;
  last_u_preDZ_ = u;

  // 4) Anti-windup
  float u_limited_for_i = fmaxf(fminf(u, (float)PWM_MAX), -(float)PWM_MAX);
  if (fabsf(u_limited_for_i) < 0.95f * PWM_MAX) {
    i_term_ += Ki_ * err * DT_;
    i_term_  = fmaxf(fminf(i_term_, (float)PWM_MAX), -(float)PWM_MAX);
  }

  // 5) Acceleration limiter on control effort + limit sign gating
  float u_target = fmaxf(fminf(u, (float)PWM_MAX), -(float)PWM_MAX);

  // Do not actively push further OUTWARD when at/over a limit
  if (pos_rev_ >= LIM_MAX_REV_ && u_target > 0) u_target = 0;
  if (pos_rev_ <= LIM_MIN_REV_ && u_target < 0) u_target = 0;

  float up_step   = ACCEL_PPS_ * DT_;
  float down_step = DECEL_PPS_ * DT_;
  float du = u_target - last_u_slew_;
  if (du > 0)      last_u_slew_ += fminf(du, up_step);
  else if (du < 0) last_u_slew_ -= fminf(-du, down_step);
  if (fabsf(last_u_slew_) < 1e-3f) last_u_slew_ = 0.0f;

  // 6) Dead-zone aware output, with final outward-drive gate
  int16_t pwm_cmd = deadzone_compensate(last_u_slew_);

  if (pos_rev_ >= LIM_MAX_REV_ && pwm_cmd > 0)  pwm_cmd = 0;
  if (pos_rev_ <= LIM_MIN_REV_ && pwm_cmd < 0)  pwm_cmd = 0;

  last_pwm_cmd_ = pwm_cmd;
  driveBM(pwm_cmd);
}
