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
constexpr float MIN_C = 5.0f;
constexpr float MAX_C = 60.0f;

struct Frame {
  uint8_t buf[8];
};

Frame temps(float fet_c, float motor_c) {
  Frame f;
  std::memcpy(f.buf, &fet_c, 4);
  std::memcpy(f.buf + 4, &motor_c, 4);
  return f;
}

HandFan::Controller make() { return HandFan::Controller(ON_C, OFF_C, TIMEOUT_MS, MIN_C, MAX_C); }

// The command bytes the can-bridge's Relay::hand_fan_set sends for 30.00 / 28.50 C.
// tests/firmware/native/test_platform_relay.cpp asserts the same bytes from the other side.
constexpr uint8_t CMD_ON_30_00_28_50[8] = {1, 0xB8, 0x0B, 0x22, 0x0B, 0, 0, 0};

Frame command(uint8_t mode, int on_centi, int off_centi) {
  Frame f{};
  f.buf[0] = mode;
  f.buf[1] = static_cast<uint8_t>(on_centi & 0xFF);
  f.buf[2] = static_cast<uint8_t>((on_centi >> 8) & 0xFF);
  f.buf[3] = static_cast<uint8_t>(off_centi & 0xFF);
  f.buf[4] = static_cast<uint8_t>((off_centi >> 8) & 0xFF);
  return f;
}

bool send(HandFan::Controller &c, uint8_t mode, int on_centi, int off_centi) {
  const Frame f = command(mode, on_centi, off_centi);
  return c.on_command_frame(f.buf, 8, false);
}

int16_t i16_at(const uint8_t *s, int at) {
  return static_cast<int16_t>(uint16_t(s[at]) | (uint16_t(s[at + 1]) << 8));
}

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

TEST_CASE("FORCE_ON and FORCE_OFF ignore the temperature, and AUTO then decides from a current reading") {
  auto c = make();
  CHECK(send(c, HandFan::MODE_FORCE_ON, 3500, 3400));
  CHECK(c.update(0));  // no frame ever heard: forced still means forced

  feed(c, 40.0f, 100);
  CHECK(send(c, HandFan::MODE_FORCE_OFF, 3500, 3400));
  CHECK_FALSE(c.update(100));

  CHECK(send(c, HandFan::MODE_AUTO, 3500, 3400));
  CHECK(c.update(100));  // the 40 C reading was tracked while forced off
}

TEST_CASE("forced modes outlast the reading timeout") {
  auto c = make();
  CHECK(send(c, HandFan::MODE_FORCE_ON, 3500, 3400));
  feed(c, 20.0f, 0);
  CHECK(c.update(TIMEOUT_MS * 10));
}

TEST_CASE("runtime thresholds move the AUTO decision") {
  auto c = make();
  feed(c, 32.0f, 0);
  CHECK_FALSE(c.update(0));
  CHECK(send(c, HandFan::MODE_AUTO, 3000, 2850));
  feed(c, 32.0f, 100);
  CHECK(c.update(100));
}

TEST_CASE("a command the Platform refuses changes nothing") {
  auto c = make();
  feed(c, 34.5f, 0);
  CHECK_FALSE(send(c, 3, 3000, 2800));        // unknown mode
  CHECK_FALSE(send(c, HandFan::MODE_FORCE_ON, 2800, 2800));  // no gap
  CHECK_FALSE(send(c, HandFan::MODE_FORCE_ON, 2800, 3000));  // inverted
  CHECK_FALSE(send(c, HandFan::MODE_FORCE_ON, 6001, 3000));  // above the top bound
  CHECK_FALSE(send(c, HandFan::MODE_FORCE_ON, 3000, 499));   // below the bottom bound
  const Frame f = command(HandFan::MODE_FORCE_ON, 3000, 2800);
  CHECK_FALSE(c.on_command_frame(f.buf, 7, false));
  CHECK_FALSE(c.on_command_frame(f.buf, 8, true));

  uint8_t s[8];
  c.encode_status(s);
  CHECK(s[0] == HandFan::MODE_AUTO);
  CHECK(i16_at(s, 2) == 3500);
  CHECK(i16_at(s, 4) == 3400);
  CHECK_FALSE(c.update(0));
}

TEST_CASE("the can-bridge's command bytes decode as sent") {
  auto c = make();
  CHECK(c.on_command_frame(CMD_ON_30_00_28_50, 8, false));
  uint8_t s[8];
  c.encode_status(s);
  CHECK(s[0] == HandFan::MODE_FORCE_ON);
  CHECK(i16_at(s, 2) == 3000);
  CHECK(i16_at(s, 4) == 2850);
}

TEST_CASE("status frame layout, which rpc_args.decode_hand_fan_status reads") {
  auto c = make();
  uint8_t s[8];

  c.encode_status(s);
  const uint8_t no_reading[8] = {0, 0x00, 0xAC, 0x0D, 0x48, 0x0D, 0x00, 0x00};
  CHECK(std::memcmp(s, no_reading, 8) == 0);

  feed(c, 41.25f, 0);
  c.update(0);
  c.encode_status(s);
  const uint8_t on_at_41_25[8] = {0, 0x03, 0xAC, 0x0D, 0x48, 0x0D, 0x1D, 0x10};
  CHECK(std::memcmp(s, on_at_41_25, 8) == 0);

  c.update(TIMEOUT_MS + 1);  // the reading has gone stale
  c.encode_status(s);
  CHECK(s[1] == 0x00);
  CHECK(i16_at(s, 6) == 0);
}

TEST_CASE("a status temperature saturates instead of overflowing the i16") {
  auto c = make();
  uint8_t s[8];
  feed(c, 1000.0f, 0);
  c.encode_status(s);
  CHECK(i16_at(s, 6) == 32767);
  feed(c, -1000.0f, 1);
  c.encode_status(s);
  CHECK(i16_at(s, 6) == -32768);
}
