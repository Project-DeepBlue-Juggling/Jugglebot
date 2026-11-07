// =============================================================================
//  test_platform_hand_fan.cpp — compiled test of the shipped Hand fan logic
// =============================================================================
//  Includes the Platform Teensy's own hand_fan.h. The thresholds below are the
//  test's, not the YAML's, so a retune of the config cannot change what this
//  asserts; tests/firmware/test_platform_hand_fan_xref.py pins the wiring and
//  the config.

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "../../../ros_ws/src/jugglebot/Teensy_code_platform/hand_fan.h"

namespace {

constexpr float ON_C = 35.0f;
constexpr float OFF_C = 34.0f;
constexpr uint32_t TIMEOUT_MS = 5000;

struct Frame {
  uint8_t buf[8];
};

Frame temps(float fet_c, float motor_c) {
  Frame f;
  std::memcpy(f.buf, &fet_c, 4);
  std::memcpy(f.buf + 4, &motor_c, 4);
  return f;
}

HandFan::Controller make() { return HandFan::Controller(ON_C, OFF_C, TIMEOUT_MS); }

void feed(HandFan::Controller &c, float motor_c, uint32_t now_ms) {
  const Frame f = temps(20.0f, motor_c);
  c.on_temps_frame(f.buf, 8, false, now_ms);
}

}  // namespace

TEST_CASE("off until a reading says otherwise, including with no frames at all") {
  auto c = make();
  CHECK_FALSE(c.update(0));
  CHECK_FALSE(c.update(1000000));
}

TEST_CASE("switches on strictly above the threshold") {
  auto c = make();
  feed(c, 35.0f, 0);
  CHECK_FALSE(c.update(0));
  feed(c, 35.01f, 100);
  CHECK(c.update(100));
}

TEST_CASE("the band between the thresholds keeps whichever state the fan is in") {
  auto c = make();
  feed(c, 34.5f, 0);
  CHECK_FALSE(c.update(0));

  feed(c, 36.0f, 100);
  CHECK(c.update(100));
  feed(c, 34.5f, 200);
  CHECK(c.update(200));
  feed(c, 34.0f, 300);
  CHECK(c.update(300));
  feed(c, 33.99f, 400);
  CHECK_FALSE(c.update(400));
}

TEST_CASE("reads the motor thermistor, never the FET temperature") {
  auto c = make();
  const Frame hot_fet = temps(90.0f, 20.0f);
  c.on_temps_frame(hot_fet.buf, 8, false, 0);
  CHECK_FALSE(c.update(0));

  const Frame hot_motor = temps(20.0f, 40.0f);
  c.on_temps_frame(hot_motor.buf, 8, false, 100);
  CHECK(c.update(100));
}

TEST_CASE("silence turns a running fan off only after the timeout") {
  auto c = make();
  feed(c, 40.0f, 1000);
  CHECK(c.update(1000));
  CHECK(c.update(1000 + TIMEOUT_MS));
  CHECK_FALSE(c.update(1000 + TIMEOUT_MS + 1));

  SUBCASE("and a later reading above the threshold brings it back") {
    feed(c, 40.0f, 9000);
    CHECK(c.update(9000));
  }
  SUBCASE("and a later reading in the band does not") {
    feed(c, 34.5f, 9000);
    CHECK_FALSE(c.update(9000));
  }
}

TEST_CASE("the timeout survives the millis() wrap") {
  auto c = make();
  const uint32_t just_before_wrap = 0xFFFFFF00u;
  feed(c, 40.0f, just_before_wrap);
  CHECK(c.update(just_before_wrap + 1000u));  // wrapped, 1 s of silence
  CHECK_FALSE(c.update(just_before_wrap + TIMEOUT_MS + 1u));
}

TEST_CASE("frames with no usable reading change nothing and do not refresh the timeout") {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  auto c = make();
  feed(c, 40.0f, 0);

  const Frame cold = temps(20.0f, 10.0f);
  c.on_temps_frame(cold.buf, 8, true, 1000);   // remote request: buf is not a reading
  CHECK(c.update(1000));
  c.on_temps_frame(cold.buf, 7, false, 2000);  // short frame
  CHECK(c.update(2000));
  feed(c, nan, 3000);
  CHECK(c.update(3000));
  feed(c, -inf, 4000);
  CHECK(c.update(4000));

  // Only the reading at t = 0 counted, so the timeout runs from there.
  CHECK_FALSE(c.update(TIMEOUT_MS + 1));
}

TEST_CASE("a NaN reading cannot start the fan") {
  auto c = make();
  feed(c, std::numeric_limits<float>::quiet_NaN(), 0);
  CHECK_FALSE(c.update(0));
}
