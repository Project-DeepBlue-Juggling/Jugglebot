/*****************************************************************************************
 *  hand_fan.h — when the Hand motor's cooling fan should run (Platform Teensy).
 *
 *  Pure logic with no Arduino dependency, so tests/firmware/native/ can run the shipped
 *  code. The sketch owns the pin, the clock and the CAN writes.
 *
 *  INPUT. The Hand ODrive broadcasts Get_Temperature by itself (cyclic message,
 *  `temperature_msg_rate_ms: 100` in config/ODrive config Files/odrive_pro_hand_config.json),
 *  so AUTO needs neither ROS2, nor the Jetson, nor the can-bridge to request anything.
 *  Payload: f32 LE FET temperature @0, f32 LE motor thermistor @4, both deg C.
 *
 *  THE AUTO RULE. On above `on_above_c`, off below `off_below_c`, unchanged between them;
 *  the gap stops the fan chattering on a reading that sits at the threshold. A frame that
 *  carries no usable reading (a remote request, a short frame, NaN or infinity from a
 *  disabled thermistor) changes nothing and does not count as hearing the Hand ODrive.
 *  With no usable reading for `timeout_ms` the fan goes off. That rides through a short
 *  ODrive reboot, and it leaves the ODrive's own motor thermal limits as the protection,
 *  which do not depend on this fan or on this bus.
 *
 *  RUNTIME CONTROL, for A/B tests. FORCE_ON and FORCE_OFF ignore the temperature, which is
 *  still tracked so a return to AUTO decides from a current reading. Mode and thresholds
 *  live in RAM only: every reboot returns to AUTO at the configured defaults, so a test
 *  value can never outlive the board's next power cycle.
 *
 *  WIRE CONTRACT (little-endian; mirrored by the can-bridge's Relay::hand_fan_set and the
 *  host's rpc_args.decode_hand_fan_status). Temperatures are i16 hundredths of a deg C.
 *    Command, PlatformCanId::HAND_FAN_CMD, dlc 8
 *      [mode u8][on_above i16 @1][off_below i16 @3][0,0,0]
 *      A command that fails validation changes nothing; the status frame sent in reply
 *      is how the host sees that.
 *    Status, PlatformCanId::HAND_FAN_STATUS, dlc 8, sent on a timer, on every command and
 *      on every fan change
 *      [mode u8][flags u8][on_above i16 @2][off_below i16 @4][motor i16 @6, 0 if invalid]
 *      flags bit0 = fan on, bit1 = a usable reading is current
 *****************************************************************************************/
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

namespace HandFan {

enum Mode : uint8_t { MODE_AUTO = 0, MODE_FORCE_ON = 1, MODE_FORCE_OFF = 2 };

constexpr uint8_t TEMPS_FRAME_LEN = 8;
constexpr uint8_t MOTOR_TEMP_OFFSET = 4;
constexpr uint8_t COMMAND_FRAME_LEN = 8;
constexpr uint8_t STATUS_FRAME_LEN = 8;
constexpr uint8_t STATUS_FLAG_FAN_ON = 0x01;
constexpr uint8_t STATUS_FLAG_READING_VALID = 0x02;

class Controller {
 public:
  constexpr Controller(float on_above_c, float off_below_c, uint32_t timeout_ms,
                       float threshold_min_c, float threshold_max_c)
      : on_above_c_(on_above_c),
        off_below_c_(off_below_c),
        timeout_ms_(timeout_ms),
        threshold_min_c_(threshold_min_c),
        threshold_max_c_(threshold_max_c) {}

  /* Feed one frame already matched on (Hand node, Get_Temperature). */
  void on_temps_frame(const uint8_t *buf, uint8_t len, bool remote, uint32_t now_ms) {
    if (remote || len != TEMPS_FRAME_LEN) return;

    float motor_c;
    std::memcpy(&motor_c, buf + MOTOR_TEMP_OFFSET, sizeof(motor_c));
    if (!std::isfinite(motor_c)) return;

    heard_ = true;
    motor_c_ = motor_c;
    last_reading_ms_ = now_ms;
    if (motor_c > on_above_c_) auto_on_ = true;
    else if (motor_c < off_below_c_) auto_on_ = false;
  }

  /* Feed one HAND_FAN_CMD frame. Returns whether it was applied. */
  bool on_command_frame(const uint8_t *buf, uint8_t len, bool remote) {
    if (remote || len != COMMAND_FRAME_LEN) return false;
    if (buf[0] > MODE_FORCE_OFF) return false;

    const float on_above_c = read_centi(buf + 1);
    const float off_below_c = read_centi(buf + 3);
    if (!(off_below_c >= threshold_min_c_ && on_above_c <= threshold_max_c_
          && off_below_c < on_above_c)) {
      return false;
    }

    mode_ = static_cast<Mode>(buf[0]);
    on_above_c_ = on_above_c;
    off_below_c_ = off_below_c;
    return true;
  }

  /* Call regularly; returns whether the fan should be running. Unsigned subtraction,
   * so the timeout holds across the 49.7-day millis() wrap. */
  bool update(uint32_t now_ms) {
    if (heard_ && (now_ms - last_reading_ms_) > timeout_ms_) {
      heard_ = false;
      auto_on_ = false;
    }
    return output();
  }

  void encode_status(uint8_t *out) const {
    out[0] = mode_;
    out[1] = (output() ? STATUS_FLAG_FAN_ON : 0u) | (heard_ ? STATUS_FLAG_READING_VALID : 0u);
    write_centi(out + 2, on_above_c_);
    write_centi(out + 4, off_below_c_);
    write_centi(out + 6, heard_ ? motor_c_ : 0.0f);
  }

 private:
  bool output() const {
    if (mode_ == MODE_FORCE_ON) return true;
    if (mode_ == MODE_FORCE_OFF) return false;
    return auto_on_;
  }

  static float read_centi(const uint8_t *p) {
    const int16_t centi = static_cast<int16_t>(uint16_t(p[0]) | (uint16_t(p[1]) << 8));
    return centi / 100.0f;
  }

  /* Saturates: a stalled motor has read 95.7 C and a broken thermistor can read anything,
   * and an out-of-range float to int16 conversion is undefined. */
  static void write_centi(uint8_t *p, float c) {
    const float centi = std::round(c * 100.0f);
    const int16_t v = centi >= 32767.0f ? int16_t(32767)
                      : centi <= -32768.0f ? int16_t(-32768)
                                           : static_cast<int16_t>(centi);
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  }

  float on_above_c_;
  float off_below_c_;
  uint32_t timeout_ms_;
  float threshold_min_c_;
  float threshold_max_c_;
  float motor_c_ = 0.0f;
  uint32_t last_reading_ms_ = 0;
  Mode mode_ = MODE_AUTO;
  bool heard_ = false;
  bool auto_on_ = false;
};

}  // namespace HandFan
