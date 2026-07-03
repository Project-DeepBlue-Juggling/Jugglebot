// =============================================================================
//  test_leg_interp.cpp — compiled BEHAVIOUR test of the REAL 500 Hz interpolator
// =============================================================================
//  Drives the actual compiled leg_interp.cpp (it #includes the .cpp so it can
//  call the static interp_isr() and reach the file-statics) and asserts the
//  safety-relevant BEHAVIOURS:
//
//    * the lead clamp (never run more than MAX_LEAD_REV ahead of the encoder),
//    * the per-leg stroke clamp (the physical backstop), with vel/torque zeroed,
//    * the present-axis TX gate (stream only to legs physically on the bus, and
//      only when output is enabled),
//    * the Hermite / Taylor / velocity-decay mode transitions, and
//    * the deferred-stow profiled descent reaching the off pose + completing.
//
//  aarch64/x86 host float is true IEEE-32 (closer to the Teensy FPU than the
//  float64 Python mirror), but these assert BEHAVIOUR — clamps fired, modes
//  transitioned, descent converged — NOT bit-exact float equality. The float64
//  numerical xref stays in tests/firmware/test_hermite_xref.py.
//
//  SCOPE: validates DECISION LOGIC, not the 500 Hz ISR timing/jitter. See README.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "axis_state.h"
#include "odrive_protocol.h"
#include "canbridge_config.h"
#include "udp_protocol.h"
#include "fake_hal.h"

#include "leg_interp.cpp"   // the unit under test (reach statics + static interp_isr)

using namespace CanBridge;

static constexpr uint8_t CMD_SETPOS = ODriveCmd::set_input_pos;   // 0x0C

static void reset_interp_test() {
  fake_reset();
  interp_reset();
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    axes[i].pos_rev = 0.0f;
    axes[i].vel_rps = 0.0f;
    axes[i].pos_timestamp_us = 0;
    axes[i].heartbeat_seen = false;
    axes[i].target_pos_rev = 0.0f;
    axes[i].target_vel_rps = 0.0f;
    axes[i].target_torque_Nm = 0.0f;
  }
  fake_set_clock(1'000'000, 1'000'000);
}

// Stage one setpoint. u1==nullptr → no next waypoint (flags bit0 clear → Taylor/
// decay extrapolation modes); else cubic Hermite between u0 and u1. `seq` drives the
// wrap-safe monotonic-seq guard (Flash-A item 5); default 0 (fresh after a reset,
// which clears the last-accepted-seq state, so the first frame always latches).
static void stage(const float u0[6], const float* u1, const float v0[6], const float accel[6],
                  uint16_t seq = 0) {
  JbUdp::SetpointPayload sp;
  memset(&sp, 0, sizeof(sp));
  for (int i = 0; i < 6; ++i) {
    sp.u0[i] = u0[i];
    if (v0)    sp.v0[i] = v0[i];
    if (accel) sp.accel[i] = accel[i];
    if (u1)    sp.u1[i] = u1[i];
  }
  sp.flags = u1 ? 0x1u : 0x0u;
  interp_on_setpoint(seq, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
}

TEST_CASE("lead clamp: command never runs more than MAX_LEAD_REV ahead of encoder") {
  reset_interp_test();
  axes[0].pos_rev = 1.0f;                       // encoder
  float u0[6] = {2.0f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f};  // leg0 commanded far ahead
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  stage(u0, nullptr, zeros, zeros);
  interp_isr();                                 // dt≈0 → cmd≈u0, then lead-clamped
  // Clamped to encoder + MAX_LEAD_REV; vel zeroed because the clamp engaged.
  CHECK(axes[0].target_pos_rev == doctest::Approx(1.0f + MAX_LEAD_REV).epsilon(0.01));
  CHECK(std::fabs(axes[0].target_pos_rev - axes[0].pos_rev) <= MAX_LEAD_REV + 1e-4f);
  CHECK(axes[0].target_vel_rps == doctest::Approx(0.0f));
}

TEST_CASE("stroke clamp: command pinned to [STROKE_MIN, STROKE_MAX], vel/torque zeroed") {
  // Above max: encoder near the top, commanded past the backstop.
  reset_interp_test();
  axes[0].pos_rev = 3.9f;
  float hi[6] = {5.0f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  stage(hi, nullptr, zeros, zeros);
  interp_isr();
  CHECK(axes[0].target_pos_rev == doctest::Approx(STROKE_MAX_REV[0]));
  CHECK(axes[0].target_vel_rps == doctest::Approx(0.0f));
  CHECK(axes[0].target_torque_Nm == doctest::Approx(0.0f));

  // Below min: encoder at 0, commanded below the bottom backstop.
  reset_interp_test();
  axes[0].pos_rev = 0.0f;
  float lo[6] = {-5.0f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f};
  stage(lo, nullptr, zeros, zeros);
  interp_isr();
  CHECK(axes[0].target_pos_rev == doctest::Approx(STROKE_MIN_REV[0]));
  CHECK(axes[0].target_vel_rps == doctest::Approx(0.0f));
}

TEST_CASE("present-axis TX gate: stream only to present legs, only when enabled") {
  reset_interp_test();
  axes[0].heartbeat_seen = true;                // present
  axes[1].heartbeat_seen = true;                // present
  // legs 2-5 absent
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};

  SUBCASE("output disabled → nothing on the wire") {
    interp_set_output_enabled(false);
    stage(u0, nullptr, zeros, zeros);
    fake_clear_sent();
    interp_isr();
    CHECK(fake_sent_count() == 0);
  }
  SUBCASE("output enabled → exactly the two present legs") {
    interp_set_output_enabled(true);
    stage(u0, nullptr, zeros, zeros);
    fake_clear_sent();
    interp_isr();
    CHECK(fake_sent_count_cmd(CMD_SETPOS) == 2);
    bool axes_seen[7] = {false};
    for (size_t i = 0; i < fake_sent_count(); ++i)
      axes_seen[ODrive::axis_of(fake_sent_at(i).id)] = true;
    CHECK(axes_seen[0]);
    CHECK(axes_seen[1]);
    CHECK(axes_seen[2] == false);               // absent leg never streamed
  }
}

TEST_CASE("Hermite mode: command tracks u0 → u1 across the segment") {
  reset_interp_test();
  axes[0].pos_rev = 0.15f;                       // encoder mid-way (no lead clamp)
  float u0[6] = {0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f};
  float u1[6] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  stage(u0, u1, zeros, zeros);
  interp_isr();                                  // dt≈0 → s=0 → cmd≈u0
  CHECK(axes[0].target_pos_rev == doctest::Approx(0.1f).epsilon(0.02));
  fake_advance((uint64_t)(SEGMENT_T_S * 1e6f));  // dt≈SEG_T → s=1 → cmd≈u1
  interp_isr();
  CHECK(axes[0].target_pos_rev == doctest::Approx(0.2f).epsilon(0.02));
}

TEST_CASE("Taylor extrapolation mode: position advances by v0*dt") {
  reset_interp_test();
  axes[0].pos_rev = 0.5f;
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float v0[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // 1 rev/s on leg 0
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  stage(u0, nullptr, v0, zeros);                 // no u1 → extrapolation
  interp_isr();                                  // dt≈0
  CHECK(axes[0].target_pos_rev == doctest::Approx(0.5f).epsilon(0.02));
  fake_advance(20'000);                          // dt=0.02 s ≤ MAX_EXTRAP_DT_S
  interp_isr();
  CHECK(axes[0].target_pos_rev == doctest::Approx(0.52f).epsilon(0.02));  // 0.5 + 1.0*0.02
  CHECK(axes[0].target_vel_rps == doctest::Approx(1.0f).epsilon(0.05));
}

TEST_CASE("velocity-decay mode: beyond MAX_EXTRAP_DT the velocity decays to zero") {
  reset_interp_test();
  axes[0].pos_rev = 0.55f;                       // near the extrapolated position (no lead clamp)
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float v0[6] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  stage(u0, nullptr, v0, zeros);
  // dt past MAX_EXTRAP_DT_S + EXTRAP_DECAY_DT_S → decay factor 0 → velocity 0.
  fake_advance((uint64_t)((MAX_EXTRAP_DT_S + EXTRAP_DECAY_DT_S) * 1e6f));
  interp_isr();
  CHECK(std::fabs(axes[0].target_vel_rps) < 0.05f);
}

TEST_CASE("deferred-stow descent: converges to the off pose, completes, present-gated") {
  reset_interp_test();
  axes[0].heartbeat_seen = true;                 // only leg 0 present
  axes[0].pos_rev = 1.0f;                        // start the descent from 1.0 rev
  interp_set_output_enabled(true);
  interp_begin_stow();                           // captures pos_rev as the descent start
  CHECK(interp_stow_active());
  CHECK(interp_stow_complete() == false);

  // Drive the descent. With a 2 ms tick and a ~2.5 rev/s limit, ~1.0 rev takes a
  // few hundred ticks; bound it generously and assert convergence + completion.
  int ticks = 0;
  while (!interp_stow_complete() && ticks < 2000) {
    fake_advance(INTERP_PERIOD_US);
    interp_isr();
    ++ticks;
  }
  CHECK(interp_stow_complete());
  CHECK(axes[0].target_pos_rev == doctest::Approx(STOW_OFF_POSE_REV).epsilon(0.02));
  // Descent streamed only to the present leg; absent legs never on the wire.
  bool absent_streamed = false;
  for (size_t i = 0; i < fake_sent_count(); ++i)
    if (ODrive::axis_of(fake_sent_at(i).id) != 0) absent_streamed = true;
  CHECK(absent_streamed == false);
}

// =============================================================================
//  Item 14 — the 500 Hz trajectory phase reads the MONOTONIC clock
// =============================================================================
//  dt = micros64() - s_base_ts_us, and s_base_ts_us was stamped with micros64() at
//  recv. A wall-clock STEP (set_wall_anchor NTP re-acquisition) must NOT move the
//  interpolated command — otherwise the commanded position jumps and jerks the
//  legs. Latch at mono=M0, extrapolate a known dt, then step the WALL clock only
//  (mono frozen) and re-tick: the command must be bit-for-bit the pre-step value.
TEST_CASE("wall step does not perturb the 500 Hz trajectory phase (item 14)") {
  reset_interp_test();
  const uint64_t W0 = 2'000'000'000ULL;   // wall base (>> the 5 s step, no underflow)
  const uint64_t M0 =    10'000'000ULL;   // mono base (independent of wall)
  fake_set_clock(W0, M0);

  axes[0].pos_rev = 0.5f;                  // encoder near the extrapolated pos (no lead clamp)
  interp_set_output_enabled(false);
  float u0[6]    = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float v0[6]    = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // 1 rev/s on leg 0
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  stage(u0, nullptr, v0, zeros);           // recv = micros64() = M0 (Taylor extrapolation mode)

  fake_advance(20'000);                    // dt = 0.02 s (both clocks move together)
  interp_isr();                            // latch + one tick at dt = 0.02 s
  const float pos_before = axes[0].target_pos_rev;
  const float vel_before = axes[0].target_vel_rps;
  REQUIRE(pos_before == doctest::Approx(0.52f).epsilon(0.02));   // 0.5 + 1.0*0.02

  // STEP the wall clock BACKWARD 5 s, mono frozen → dt is recomputed from mono and
  // is unchanged, so the command must be identical.
  fake_set_clock(fake_wall_us() - 5'000'000ULL, fake_mono_us());
  interp_isr();
  CHECK(axes[0].target_pos_rev == doctest::Approx(pos_before));
  CHECK(axes[0].target_vel_rps == doctest::Approx(vel_before));

  // STEP the wall clock FORWARD 10 s, mono still frozen → still identical.
  fake_set_clock(fake_wall_us() + 10'000'000ULL, fake_mono_us());
  interp_isr();
  CHECK(axes[0].target_pos_rev == doctest::Approx(pos_before));
  CHECK(axes[0].target_vel_rps == doctest::Approx(vel_before));
}

// =============================================================================
//  Flash-A item 1a — in-progress-move interlock: a cold-start move suppresses TX
// =============================================================================
//  A firmware homing / activate / deactivate move drives the SAME leg ODrives the
//  500 Hz interp ISR streams to. If both TX at once they co-drive the legs (a fight
//  that can jerk the platform). The ISR suppresses its leg TX at zero latency while
//  any cold-start move is active (FAULT_TASK_HZ = 10 Hz is too slow — up to 100 ms of
//  co-driving otherwise). The target cache STILL updates (telemetry is unaffected).
static void expect_coldstart_suppresses_tx(void (*set_active)(bool)) {
  reset_interp_test();
  axes[0].heartbeat_seen = true;                 // present leg
  axes[0].pos_rev = 0.5f;                         // encoder at the command (no lead clamp)
  interp_set_output_enabled(true);
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  stage(u0, nullptr, zeros, zeros);
  set_active(true);                               // a cold-start move begins
  fake_clear_sent();
  interp_isr();
  CHECK(fake_sent_count() == 0);                  // no leg setpoint streamed during the move
  // The target cache STILL updates (telemetry unaffected by the interlock).
  CHECK(axes[0].target_pos_rev == doctest::Approx(0.5f).epsilon(0.02));
}

TEST_CASE("cold-start move suppresses the 500 Hz leg TX; target cache still updates (item 1a)") {
  expect_coldstart_suppresses_tx(fake_set_homing);
  expect_coldstart_suppresses_tx(fake_set_activate);
  expect_coldstart_suppresses_tx(fake_set_deactivate);

  // Control: with NO cold-start move active, the same setup DOES stream (proves the
  // suppression above is caused by the interlock, not some other gate).
  reset_interp_test();
  axes[0].heartbeat_seen = true;
  axes[0].pos_rev = 0.5f;
  interp_set_output_enabled(true);
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  stage(u0, nullptr, zeros, zeros);
  fake_clear_sent();
  interp_isr();
  CHECK(fake_sent_count_cmd(CMD_SETPOS) == 1);    // the one present leg is streamed
}

// =============================================================================
//  Flash-A item 5 — setpoint trust boundary: isfinite drop + wrap-safe seq guard
// =============================================================================

TEST_CASE("a setpoint with a non-finite field is DROPPED (never staged / streamed), item 5") {
  reset_interp_test();
  axes[0].heartbeat_seen = true;
  interp_set_output_enabled(true);
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  u0[0] = std::nanf("");                          // a NaN in u0[0]
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  fake_clear_sent();
  stage(u0, nullptr, zeros, zeros);               // seq 0, first frame → seq guard passes, isfinite drops it
  CHECK_FALSE(s_pending);                         // dropped before staging (no s_pending set)
  interp_isr();
  CHECK(fake_sent_count() == 0);                  // nothing reached the wire
  // Never latched → the target cache stays at its finite reset value.
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    CHECK(std::isfinite(axes[i].target_pos_rev));
    CHECK(std::isfinite(axes[i].target_vel_rps));
  }
  // CRITICAL: a dropped frame must NOT bump the staleness clock — a stream of all-NaN
  // frames must still eventually trip MPC_STALE (as if the link went quiet).
  CHECK(interp_last_setpoint_us() == 0);
}

TEST_CASE("setpoint seq guard is strictly-greater + wrap-safe (shared host stream counter), item 5") {
  reset_interp_test();
  float a[6] = {0.10f, 0.10f, 0.10f, 0.10f, 0.10f, 0.10f};
  float b[6] = {0.20f, 0.20f, 0.20f, 0.20f, 0.20f, 0.20f};
  float c[6] = {0.30f, 0.30f, 0.30f, 0.30f, 0.30f, 0.30f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};

  // seq 10 accepted (first frame).
  stage(a, nullptr, zeros, zeros, 10);
  CHECK(s_pending);
  interp_isr();                                   // latch → base = a
  CHECK(interp_base_pos(0) == doctest::Approx(0.10f));

  // seq 9 (stale) dropped — (int16_t)(9 - 10) = -1 <= 0.
  stage(b, nullptr, zeros, zeros, 9);
  CHECK_FALSE(s_pending);
  interp_isr();
  CHECK(interp_base_pos(0) == doctest::Approx(0.10f));   // base unchanged

  // seq 10 (duplicate) dropped — (int16_t)(10 - 10) = 0 <= 0.
  stage(b, nullptr, zeros, zeros, 10);
  CHECK_FALSE(s_pending);
  interp_isr();
  CHECK(interp_base_pos(0) == doctest::Approx(0.10f));   // base still unchanged

  // seq 11 accepted — (int16_t)(11 - 10) = 1 > 0. Proves NON-contiguous acceptance:
  // a strictly-greater guard accepts 11 even though 10→11 here skipped nothing, and
  // (critically) it would accept a setpoint whose seq jumped forward past a heartbeat.
  stage(c, nullptr, zeros, zeros, 11);
  CHECK(s_pending);
  interp_isr();
  CHECK(interp_base_pos(0) == doctest::Approx(0.30f));

  // Wrap-safety: last-accepted 0xFFFF, then 0x0000 must be ACCEPTED (not dropped) —
  // (int16_t)(0x0000 - 0xFFFF) = (int16_t)0x0001 = +1 > 0. A naive unsigned/`==last+1`
  // guard would mishandle the 16-bit wrap.
  reset_interp_test();
  stage(a, nullptr, zeros, zeros, 0xFFFF);
  CHECK(s_pending);
  interp_isr();
  CHECK(interp_base_pos(0) == doctest::Approx(0.10f));
  stage(c, nullptr, zeros, zeros, 0x0000);
  CHECK(s_pending);                               // 0x0000 accepted after 0xFFFF (wrap)
  interp_isr();
  CHECK(interp_base_pos(0) == doctest::Approx(0.30f));
}
