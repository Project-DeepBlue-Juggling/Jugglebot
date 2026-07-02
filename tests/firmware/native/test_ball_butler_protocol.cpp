// =============================================================================
//  test_ball_butler_protocol.cpp — EXECUTE the real BB encoders + emit a golden
// =============================================================================
//  ball_butler_protocol.h claims to be a "byte-for-byte port of
//  jugglebot.can.ball_butler.encode_throw_command + encode_state_command", but it
//  was the ONE canbridge firmware encoder with ZERO compiled coverage (coverage
//  gap 7) — a wrong scale, id, byte offset, or range bound passed silently. This
//  compiles + runs the real encoders and emits their exact bytes into a committed
//  golden; tests/firmware/test_ball_butler_xref.py then reproduces every VALID row
//  from the AUTHORITATIVE ball_butler.py (byte-for-byte) and independently checks
//  that BOTH sides REJECT the same invalid throws.
//
//  throw_time (bytes 6-7) depends on now_wall_us(); the driver defines a settable
//  fixed clock and the golden carries wall_us, so the Python reproduction pins the
//  same instant (integer-ms-aligned wall_us → C++ µs→ms integer-divide agrees with
//  Python's float path exactly).
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>

#include "protocol_config.h"
#include "odrive_protocol.h"
#include "time_base.h"
#include "ball_butler_protocol.h"

namespace CanBridge {
static uint64_t g_wall_us = 0;
uint64_t now_wall_us() { return g_wall_us; }   // HAL: time_base.h
uint64_t micros64()    { return g_wall_us; }
}

using namespace CanBridge;

// The golden table (VALID rows only — byte-parity). Integer-ms wall_us + values
// whose float32/float64 scaling agrees, so the Python reproduction matches all 8B.
struct ThrowRow { float yaw, pitch, speed, delay; uint64_t wall_us; };
static ThrowRow throw_rows[] = {
  {0.0f,  0.0f, 0.0f, 0.0f, 0ULL},
  {0.5f,  0.5f, 1.0f, 0.1f, 1700000000000ULL},
  {-0.5f, 0.3f, 3.0f, 0.25f, 1700000000000ULL},
  {1.0f,  1.2f, 6.5f, 1.0f, 1700000000000ULL},
};
static constexpr int N_THROW = (int)(sizeof(throw_rows) / sizeof(throw_rows[0]));

static void emit_golden(const char* path) {
  FILE* fp = fopen(path, "w");
  if (!fp) { fprintf(stderr, "cannot open %s\n", path); return; }
  fprintf(fp, "[\n");
  for (int i = 0; i < N_THROW; ++i) {
    const ThrowRow& r = throw_rows[i];
    g_wall_us = r.wall_us;
    const auto f = CanBridge::BallButler::encode_throw(r.yaw, r.pitch, r.speed, r.delay);
    char hex[17] = {0};
    for (int b = 0; b < f.len; ++b) snprintf(hex + b * 2, 3, "%02x", f.buf[b]);
    fprintf(fp,
      "  {\"yaw\": %.9g, \"pitch\": %.9g, \"speed\": %.9g, \"delay\": %.9g, "
      "\"wall_us\": %llu, \"id\": %u, \"len\": %u, \"data\": \"%s\"}%s\n",
      r.yaw, r.pitch, r.speed, r.delay, (unsigned long long)r.wall_us,
      f.id, f.len, hex, i + 1 < N_THROW ? "," : "");
  }
  fprintf(fp, "]\n");
  fclose(fp);
}

// ── Self-checks ───────────────────────────────────────────────────────────────

TEST_CASE("encode_throw: id 0x7D0, len 8, byte offsets + scaling") {
  g_wall_us = 1700000000000ULL;
  const auto f = CanBridge::BallButler::encode_throw(0.5f, 0.5f, 1.0f, 0.1f);
  CHECK(f.id == BallButlerCanId::THROW_CMD);
  CHECK(f.len == 8);
  int16_t yaw; uint16_t pitch, speed;
  memcpy(&yaw, &f.buf[0], 2); memcpy(&pitch, &f.buf[2], 2); memcpy(&speed, &f.buf[4], 2);
  CHECK(yaw == (int16_t)(0.5f * CanBridge::BallButler::YAW_SCALE));
  CHECK(pitch == (uint16_t)(0.5f * CanBridge::BallButler::PITCH_SCALE));
  CHECK(speed == (uint16_t)(1.0f * CanBridge::BallButler::SPEED_SCALE));   // 10000
}

TEST_CASE("throw_args_valid accepts in-range and rejects each out-of-range + NaN/Inf") {
  CHECK(CanBridge::BallButler::throw_args_valid(0.0f, 0.5f, 3.0f, 0.1f));
  CHECK(CanBridge::BallButler::throw_args_valid(-3.14f, 0.0f, 0.0f, 0.0f));
  // yaw == +pi is the EXCLUSIVE upper bound → reject.
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(CanBridge::BallButler::YAW_MAX_RAD, 0.5f, 3.0f, 0.1f));
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(4.0f, 0.5f, 3.0f, 0.1f));      // yaw over
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(0.0f, 2.0f, 3.0f, 0.1f));      // pitch over
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(0.0f, -0.1f, 3.0f, 0.1f));     // pitch under
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(0.0f, 0.5f, 100.0f, 0.1f));    // speed over
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(0.0f, 0.5f, 3.0f, 100.0f));    // delay over
  const float nan = nanf(""), inf = INFINITY;
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(nan, 0.5f, 3.0f, 0.1f));       // NaN
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(0.0f, inf, 3.0f, 0.1f));       // Inf
}

TEST_CASE("payloadless state commands: ids + len 0") {
  CHECK(CanBridge::BallButler::encode_reload().id == BallButlerCanId::RELOAD_CMD);
  CHECK(CanBridge::BallButler::encode_reload().len == 0);
  CHECK(CanBridge::BallButler::encode_reset().id == BallButlerCanId::RESET_CMD);
  CHECK(CanBridge::BallButler::encode_calibrate_loc().id == BallButlerCanId::CALIBRATE_LOC_CMD);
}

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--emit-golden") == 0 && i + 1 < argc) {
      emit_golden(argv[i + 1]);
      return 0;
    }
  }
  doctest::Context ctx(argc, argv);
  return ctx.run();
}
