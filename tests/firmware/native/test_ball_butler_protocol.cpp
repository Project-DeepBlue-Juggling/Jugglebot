// =============================================================================
//  test_ball_butler_protocol.cpp — EXECUTE the real BB encoders + emit a golden
// =============================================================================
//  ball_butler_protocol.h claims to be a "byte-for-byte port of
//  jugglebot.can.ball_butler.encode_throw_command + encode_state_command", but it
//  was the ONE canbridge firmware encoder with ZERO compiled coverage — a wrong scale, id, byte offset, or range bound passed silently. This
//  compiles + runs the real encoders and emits their exact bytes into a committed
//  golden; tests/firmware/test_ball_butler_xref.py then reproduces every VALID row
//  from the AUTHORITATIVE ball_butler.py (byte-for-byte) and independently checks
//  that BOTH sides REJECT the same invalid throws.
//
//  throw_time (bytes 6-7) is the ABSOLUTE release's epoch-ms low 16 bits (FW 29,
//  PROTOCOL_VERSION 10 — encode_throw takes release_wall_us and never reads the
//  clock); the golden carries release_wall_us and the Python reproduction
//  (ball_butler.encode_throw_command_at) floors µs→ms exactly as the C++ does.
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
struct ThrowRow { float yaw, pitch, speed; uint64_t release_wall_us; };
static ThrowRow throw_rows[] = {
  {0.0f,  0.0f, 0.0f, 0ULL},
  {0.5f,  0.5f, 1.0f, 1700000000100000ULL},
  {-0.5f, 0.3f, 3.0f, 1700000000250000ULL},
  {1.0f,  1.2f, 6.5f, 1700000001000999ULL},   // sub-ms part floors away
};
static constexpr int N_THROW = (int)(sizeof(throw_rows) / sizeof(throw_rows[0]));

static void emit_golden(const char* path) {
  FILE* fp = fopen(path, "w");
  if (!fp) { fprintf(stderr, "cannot open %s\n", path); return; }
  fprintf(fp, "[\n");
  for (int i = 0; i < N_THROW; ++i) {
    const ThrowRow& r = throw_rows[i];
    const auto f = CanBridge::BallButler::encode_throw(r.yaw, r.pitch, r.speed, r.release_wall_us);
    char hex[17] = {0};
    for (int b = 0; b < f.len; ++b) snprintf(hex + b * 2, 3, "%02x", f.buf[b]);
    fprintf(fp,
      "  {\"yaw\": %.9g, \"pitch\": %.9g, \"speed\": %.9g, "
      "\"release_wall_us\": %llu, \"id\": %u, \"len\": %u, \"data\": \"%s\"}%s\n",
      r.yaw, r.pitch, r.speed, (unsigned long long)r.release_wall_us,
      f.id, f.len, hex, i + 1 < N_THROW ? "," : "");
  }
  fprintf(fp, "]\n");
  fclose(fp);
}

// ── Self-checks ───────────────────────────────────────────────────────────────

TEST_CASE("encode_throw: id 0x7D0, len 8, byte offsets + scaling") {
  g_wall_us = 999ULL;   // encode_throw must NOT read the clock (FW 29)
  const auto f = CanBridge::BallButler::encode_throw(0.5f, 0.5f, 1.0f, 1700000000100000ULL);
  CHECK(f.id == BallButlerCanId::THROW_CMD);
  CHECK(f.len == 8);
  int16_t yaw; uint16_t pitch, speed;
  memcpy(&yaw, &f.buf[0], 2); memcpy(&pitch, &f.buf[2], 2); memcpy(&speed, &f.buf[4], 2);
  CHECK(yaw == (int16_t)(0.5f * CanBridge::BallButler::YAW_SCALE));
  CHECK(pitch == (uint16_t)(0.5f * CanBridge::BallButler::PITCH_SCALE));
  CHECK(speed == (uint16_t)(1.0f * CanBridge::BallButler::SPEED_SCALE));   // 10000
  uint16_t tt; memcpy(&tt, &f.buf[6], 2);
  CHECK(tt == (uint16_t)((1700000000100ULL) & 0xFFFFULL));
}

TEST_CASE("check_release: PAST / TOO_SOON / OK / TOO_FAR at the bounds, lead saturated") {
  using CanBridge::BallButler::check_release;
  using V = CanBridge::BallButler::ReleaseVerdict;
  const uint64_t now = 1700000000000000ULL;
  const int64_t MIN = CanBridge::BallButler::RELEASE_MIN_LEAD_US;
  const int64_t MAX = CanBridge::BallButler::RELEASE_MAX_LEAD_US;
  int32_t lead = 0;
  CHECK(check_release(now - 5, now, lead) == V::PAST);      CHECK(lead == -5);
  CHECK(check_release(now, now, lead) == V::PAST);          CHECK(lead == 0);
  CHECK(check_release(now + MIN - 1, now, lead) == V::TOO_SOON);
  CHECK(check_release(now + MIN, now, lead) == V::OK);      CHECK(lead == (int32_t)MIN);
  CHECK(check_release(now + MAX, now, lead) == V::OK);
  CHECK(check_release(now + MAX + 1, now, lead) == V::TOO_FAR);
  CHECK(check_release(0, now, lead) == V::PAST);            CHECK(lead == INT32_MIN);
  CHECK(check_release(now + 4000000000ULL, now, lead) == V::TOO_FAR);  CHECK(lead == INT32_MAX);
}

TEST_CASE("throw_args_valid accepts in-range and rejects each out-of-range + NaN/Inf") {
  CHECK(CanBridge::BallButler::throw_args_valid(0.0f, 0.5f, 3.0f));
  CHECK(CanBridge::BallButler::throw_args_valid(-3.14f, 0.0f, 0.0f));
  // yaw == +pi is the EXCLUSIVE upper bound → reject.
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(CanBridge::BallButler::YAW_MAX_RAD, 0.5f, 3.0f));
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(4.0f, 0.5f, 3.0f));      // yaw over
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(0.0f, 2.0f, 3.0f));      // pitch over
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(0.0f, -0.1f, 3.0f));     // pitch under
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(0.0f, 0.5f, 100.0f));    // speed over
  const float nan = nanf(""), inf = INFINITY;
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(nan, 0.5f, 3.0f));       // NaN
  CHECK_FALSE(CanBridge::BallButler::throw_args_valid(0.0f, inf, 3.0f));       // Inf
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
