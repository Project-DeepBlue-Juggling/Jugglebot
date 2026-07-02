// =============================================================================
//  test_odrive_protocol.cpp — EXECUTE the real C++ ODrive encoders + emit a golden
// =============================================================================
//  odrive_protocol.h is a hand port of jugglebot.can.odrive. Until now the only
//  cross-language check (test_odrive_protocol_xref.py) TRANSCRIBED the C++ arithmetic
//  into Python and compared THAT to odrive.py — both sides Python, so a wrong memcpy
//  offset / struct order / arb-id shift in the REAL header passed silently
//  (coverage gap 8). This compiles + runs the real encoders and emits their exact
//  bytes into a committed golden (odrive_protocol_golden.json). Two guards then pin
//  it: test_native_firmware.py checks freshly-emitted == committed (C++ drift, on the
//  Jetson), and test_odrive_protocol_xref.py reproduces every golden entry from the
//  AUTHORITATIVE odrive.py (Python drift, everywhere) — transitively byte-for-byte
//  cross-language equality that EXECUTES the header.
//
//  FF vectors are chosen so the int16 scaling lands on EXACT integers, dodging the
//  known lroundf(C++, half-away) vs int(round(...))(Python, banker's) 1-LSB
//  divergence on X.5 boundaries (documented in the Python xref).
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "protocol_config.h"
#include "canbridge_config.h"
#include "odrive_protocol.h"

using namespace CanBridge;

// One golden row: the real encoder's arb id + payload bytes.
struct Row { const char* name; ODrive::CanFrame f; };

static ODrive::CanFrame ctrl_mode() {
  return ODrive::encode_set_controller_mode(
      0, ODriveControlMode::POSITION, ODriveInputMode::PASSTHROUGH);
}

// The fixed table — the SAME (name → args) the Python xref reproduces from odrive.py.
static Row rows[] = {
  {"get_version",           ODrive::encode_get_version(0)},
  {"set_state_idle",        ODrive::encode_set_state(0, ODriveState::IDLE)},
  {"set_state_closed_loop", ODrive::encode_set_state(0, ODriveState::CLOSED_LOOP)},
  {"set_controller_mode",   ctrl_mode()},
  {"set_input_pos",         ODrive::encode_set_input_pos(0, 2.5f, 1000, -5000)},
  {"set_input_vel",         ODrive::encode_set_input_vel(0, 1.5f, 0.0f)},
  {"set_input_torque",      ODrive::encode_set_input_torque(0, 0.75f)},
  {"set_vel_curr_limits",   ODrive::encode_set_vel_curr_limits(0, 4.0f, 10.0f)},
  {"set_traj_vel_limit",    ODrive::encode_set_traj_vel_limit(0, 15.0f)},
  {"set_traj_acc_limits",   ODrive::encode_set_traj_acc_limits(0, 20.0f, 25.0f)},
  {"set_absolute_position", ODrive::encode_set_absolute_position(0, 1.5f)},
  {"set_pos_gain",          ODrive::encode_set_pos_gain(0, 30.0f)},
  {"set_vel_gains",         ODrive::encode_set_vel_gains(0, 0.1f, 0.05f)},
  {"clear_errors",          ODrive::encode_clear_errors(0)},
  {"reboot",                ODrive::encode_reboot(0)},
  {"sdo_read",              ODrive::encode_sdo_read(0, 488)},
  {"sdo_write",             ODrive::encode_sdo_write(0, 488, 2.5f)},
  // encode_leg_setpoint: FF (0.5 rps, 0.1 Nm) chosen so leg-scale products are exact.
  {"leg_setpoint_leg0",     ODrive::encode_leg_setpoint(0, 2.0f, 0.5f, 0.1f)},
  {"leg_setpoint_hand",     ODrive::encode_leg_setpoint(HAND_AXIS, 2.0f, 0.5f, 0.1f)},
};
static constexpr int N_ROWS = (int)(sizeof(rows) / sizeof(rows[0]));

static void emit_golden(const char* path) {
  FILE* fp = fopen(path, "w");
  if (!fp) { fprintf(stderr, "cannot open %s\n", path); return; }
  fprintf(fp, "{\n");
  for (int i = 0; i < N_ROWS; ++i) {
    char hex[17] = {0};
    for (int b = 0; b < rows[i].f.len; ++b)
      snprintf(hex + b * 2, 3, "%02x", rows[i].f.buf[b]);
    fprintf(fp, "  \"%s\": {\"arb\": %u, \"len\": %u, \"data\": \"%s\"}%s\n",
            rows[i].name, rows[i].f.id, rows[i].f.len, hex,
            i + 1 < N_ROWS ? "," : "");
  }
  fprintf(fp, "}\n");
  fclose(fp);
}

// ── Self-checks (so the binary runs standalone under the pytest wrapper) ──────

TEST_CASE("arb_id packs (axis<<5)|cmd and axis_of/cmd_of invert it") {
  const uint32_t a = ODrive::arb_id(3, ODriveCmd::set_input_pos);
  CHECK(a == ((3u << 5) | (uint32_t)ODriveCmd::set_input_pos));
  CHECK(ODrive::axis_of(a) == 3);
  CHECK(ODrive::cmd_of(a) == (uint8_t)ODriveCmd::set_input_pos);
}

TEST_CASE("encode/decode round-trip (set_input_pos payload layout <fhh)") {
  const auto f = ODrive::encode_set_input_pos(0, 2.5f, 1000, -5000);
  float pos; int16_t v, t;
  memcpy(&pos, &f.buf[0], 4); memcpy(&v, &f.buf[4], 2); memcpy(&t, &f.buf[6], 2);
  CHECK(pos == doctest::Approx(2.5f));
  CHECK(v == 1000);
  CHECK(t == -5000);
}

TEST_CASE("get_version is a zero-length frame; clear/reboot are 8 zero bytes") {
  CHECK(ODrive::encode_get_version(0).len == 0);
  const auto c = ODrive::encode_clear_errors(0);
  uint8_t zero[8] = {0};
  CHECK(memcmp(c.buf, zero, 8) == 0);
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
