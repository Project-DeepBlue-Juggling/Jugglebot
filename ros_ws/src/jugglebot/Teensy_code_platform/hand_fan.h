/*****************************************************************************************
 *  hand_fan.h — when the Hand motor's cooling fan should run (Platform Teensy, FW 8).
 *
 *  Pure logic with no Arduino dependency, so tests/firmware/native/ can run the shipped
 *  code. The sketch owns the pin and the clock.
 *
 *  INPUT. The Hand ODrive broadcasts Get_Temperature by itself (cyclic message,
 *  `temperature_msg_rate_ms: 100` in config/ODrive config Files/odrive_pro_hand_config.json),
 *  so this needs neither ROS2, nor the Jetson, nor the can-bridge to request anything.
 *  Payload: f32 LE FET temperature @0, f32 LE motor thermistor @4, both deg C.
 *
 *  THE RULE. On above `on_above_c`, off below `off_below_c`, unchanged between them; the
 *  gap stops the fan chattering on a reading that sits at the threshold. A frame that
 *  carries no usable reading (a remote request, a short frame, NaN or infinity from a
 *  disabled thermistor) changes nothing and does not count as hearing the Hand ODrive.
 *  With no usable reading for `timeout_ms` the fan goes off. That rides through a short
 *  ODrive reboot, and it leaves the ODrive's own motor thermal limits as the protection,
 *  which do not depend on this fan or on this bus.
 *****************************************************************************************/
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

namespace HandFan {

constexpr uint8_t TEMPS_FRAME_LEN = 8;
constexpr uint8_t MOTOR_TEMP_OFFSET = 4;

class Controller {
 public:
  constexpr Controller(float on_above_c, float off_below_c, uint32_t timeout_ms)
      : on_above_c_(on_above_c), off_below_c_(off_below_c), timeout_ms_(timeout_ms) {}

  /* Feed one frame already matched on (Hand node, Get_Temperature). */
  void on_temps_frame(const uint8_t *buf, uint8_t len, bool remote, uint32_t now_ms) {
    if (remote || len != TEMPS_FRAME_LEN) return;

    float motor_c;
    std::memcpy(&motor_c, buf + MOTOR_TEMP_OFFSET, sizeof(motor_c));
    if (!std::isfinite(motor_c)) return;

    heard_ = true;
    last_reading_ms_ = now_ms;
    if (motor_c > on_above_c_) on_ = true;
    else if (motor_c < off_below_c_) on_ = false;
  }

  /* Call regularly; returns whether the fan should be running. Unsigned subtraction,
   * so the timeout holds across the 49.7-day millis() wrap. */
  bool update(uint32_t now_ms) {
    if (heard_ && (now_ms - last_reading_ms_) > timeout_ms_) {
      heard_ = false;
      on_ = false;
    }
    return on_;
  }

 private:
  float on_above_c_;
  float off_below_c_;
  uint32_t timeout_ms_;
  uint32_t last_reading_ms_ = 0;
  bool heard_ = false;
  bool on_ = false;
};

}  // namespace HandFan
