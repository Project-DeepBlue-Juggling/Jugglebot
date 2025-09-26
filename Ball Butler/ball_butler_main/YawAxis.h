#pragma once
#include <Arduino.h>
#include <SPI.h>

/* Reusable yaw-axis controller for a brushed DC motor + AS5047P (Teensy 4.x).
 * Auto-enable: drives only if |error| > POS_TOL_REV and not estopped.
 * No Serial printing here—use readTelemetry() from your sketch.
 */

class YawAxis {
public:
  struct Telemetry {
    bool   enabled;
    bool   estop;
    float  pos_deg;
    float  vel_rps;
    float  cmd_deg;
    float  err_deg;
    float  u;         // PID effort (pre-slew)
    float  u_slew;    // after accel limiter, before dead-zone
    int16_t pwm;      // final signed PWM command
    int32_t turn;
    uint16_t raw;
    float  accelPPS;
    float  decelPPS;
  };

  YawAxis(uint8_t csPin, uint8_t ina1, uint8_t ina2, uint8_t pwmPin,
          uint8_t pwmMin = 60, uint32_t pwmHz = 20000);

  void begin(float loopHz = 150.0f);   // starts control ISR
  void end();                          // stops control ISR, coasts

  // Targets
  void setTargetDeg(float deg);
  void setTargetRev(float rev);
  void moveRelDeg(float ddeg);
  void moveRelRev(float drev);

  // Convenience
  void zeroHere();                     // make current angle = 0 rev & hold
  void estop();                        // latched coast (won't auto-enable)
  void clearEstop();                   // clear E-stop, return to auto mode

  // Tuning
  void setGains(float kp, float ki, float kd);
  void setAccel(float accelPPS, float decelPPS); // PWM counts / sec
  void setFF(float ff_pwm);
  void setToleranceRev(float pos_tol_rev);
  void setDir(int8_t encDir, int8_t motorDir);
  void setDeadzoneMin(uint8_t pwmMin);

  // Telemetry (interrupt-safe snapshot)
  Telemetry readTelemetry() const;

  // State queries
  bool isEnabled()  const { return enabled_; }
  bool isEstopped() const { return estop_; }

private:
  // Static trampoline into the instance ISR
  static void isrTrampoline();
  void controlISR();

  // hardware helpers
  uint16_t readAS();
  void     driveBM_raw(int16_t pwm_abs, int8_t sign);
  void     driveBM(int16_t signed_pwm);
  int16_t  deadzone_compensate(float u);

private:
  // hardware config
  const uint8_t CS_PIN_, INA1_PIN_, INA2_PIN_, PWM_PIN_;
  volatile uint8_t  PWM_MIN_;
  const uint32_t PWM_HZ_;

  // control timing
  float LOOP_HZ_;
  float DT_;

  // constants
  static constexpr uint16_t AS5047_NOP = 0xFFFF;
  static constexpr float    CPR = 16384.0f;
  static constexpr float    DEG2REV = 1.0f / 360.0f;
  static constexpr float    REV2DEG = 360.0f;
  static constexpr uint8_t  PWM_MAX  = 255;

  // tuning
  volatile float Kp_ = 120.0f;
  volatile float Ki_ = 10.0f;
  volatile float Kd_ = 1.5f;
  volatile float FF_PWM_ = 20.0f;
  volatile float POS_TOL_REV_ = 0.0014f; // ~0.5 deg
  volatile float ACCEL_PPS_ = 500.0f;
  volatile float DECEL_PPS_ = 500.0f;
  volatile int8_t ENC_DIR_ = -1;
  volatile int8_t MOTOR_DIR_ = +1;

  // state
  volatile bool estop_   = false;  // latched
  volatile bool enabled_ = false;  // current drive state (auto)
  volatile float cmd_pos_rev_ = 0.0f;
  volatile float pos_rev_     = 0.0f;
  volatile float vel_rps_     = 0.0f;
  volatile float i_term_      = 0.0f;
  volatile int16_t last_pwm_cmd_ = 0;
  volatile float   last_u_preDZ_ = 0.0f;
  volatile float   last_u_slew_  = 0.0f;
  volatile float   last_err_     = 0.0f;
  volatile float   sd_accum_     = 0.0f;

  volatile int32_t  turn_count_ = 0;
  volatile uint16_t prev_raw_   = 0;
  volatile float pos_offset_rev_ = 0.0f;   // reference offset in revolutions

  // timer
  IntervalTimer timer_;

  // singleton (one instance supported)
  static YawAxis* instance_;
};
