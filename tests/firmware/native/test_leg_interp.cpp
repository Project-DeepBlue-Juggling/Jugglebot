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
// `torque` (optional) fills torque_ff — drives the ingest torque clamp tests.
static void stage(const float u0[6], const float* u1, const float v0[6], const float accel[6],
                  uint16_t seq = 0, const float* torque = nullptr) {
  JbUdp::SetpointPayload sp;
  memset(&sp, 0, sizeof(sp));
  for (int i = 0; i < 6; ++i) {
    sp.u0[i] = u0[i];
    if (v0)     sp.v0[i] = v0[i];
    if (accel)  sp.accel[i] = accel[i];
    if (u1)     sp.u1[i] = u1[i];
    if (torque) sp.torque_ff[i] = torque[i];
  }
  sp.flags = u1 ? 0x1u : 0x0u;
  interp_on_setpoint(seq, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
}

TEST_CASE("lead clamp: bounds position, KEEPS vel_ff (capped), sets the clamp mask") {
  // 2026-07-10 forensics: the lead clamp no longer ZEROES vel_ff when it engages —
  // that manufactured the ~6 Hz bang-bang stutter. It now passes the true
  // interpolated vel_ff through, bounded to ±LEAD_CLAMP_VELFF_LIMIT_RPS, and reports
  // engagement via interp_lead_clamp_mask().
  float zeros[6] = {0, 0, 0, 0, 0, 0};

  SUBCASE("position bounded to encoder ± MAX_LEAD_REV; vel_ff preserved (under cap)") {
    reset_interp_test();
    axes[0].pos_rev = 1.0f;                       // encoder
    float u0[6] = {2.0f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f};  // leg0 commanded far ahead
    float v0[6] = {2.0f, 0, 0, 0, 0, 0};          // real +2 rev/s feedforward
    stage(u0, nullptr, v0, zeros);
    interp_isr();                                 // dt≈0 → cmd≈u0, then lead-clamped
    CHECK(axes[0].target_pos_rev == doctest::Approx(1.0f + MAX_LEAD_REV).epsilon(0.01));
    CHECK(std::fabs(axes[0].target_pos_rev - axes[0].pos_rev) <= MAX_LEAD_REV + 1e-4f);
    // vel_ff NOT zeroed — the true 2.0 rev/s (< 3.5 cap) survives the clamp.
    CHECK(axes[0].target_vel_rps == doctest::Approx(2.0f));
    CHECK((interp_lead_clamp_mask() & 0x1u) != 0);   // leg0 clamped
    CHECK((interp_lead_clamp_mask() & 0x2u) == 0);   // leg1 not clamped
  }

  SUBCASE("vel_ff magnitude is capped at LEAD_CLAMP_VELFF_LIMIT_RPS") {
    reset_interp_test();
    axes[0].pos_rev = 1.0f;
    float u0[6] = {2.0f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f};
    float v0[6] = {9.0f, 0, 0, 0, 0, 0};          // over-limit feedforward
    stage(u0, nullptr, v0, zeros);
    interp_isr();
    CHECK(axes[0].target_vel_rps == doctest::Approx(LEAD_CLAMP_VELFF_LIMIT_RPS));
  }

  SUBCASE("clamp mask clears when the command tracks the encoder") {
    reset_interp_test();
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.5f;   // all encoders aligned
    float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};       // commanded == encoder → no clamp
    stage(u0, nullptr, zeros, zeros);
    interp_isr();
    CHECK(interp_lead_clamp_mask() == 0);
  }
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
//  2026-07-14 gravity-FF firmware sitting — the torque_ff ingest clamp
// =============================================================================
//  interp_on_setpoint bounds |torque_ff[i]| to Dynamics::TORQUE_FF_FIRMWARE_
//  CLAMP_WIRE_NM (wire-Nm) per leg BEFORE staging, and publishes a per-leg
//  engagement mask (interp_torque_clamp_mask, mirrored onto HeartbeatT2J flags
//  bits 8-13). CLAMP-not-reject: an oversized torque with valid pos/vel must
//  degrade to a bounded torque, never starve the interp into an MPC_STALE
//  E-STOP mid-motion. A NaN torque still drops the WHOLE frame (isfinite gate).

TEST_CASE("torque_ff ingest clamp: binds at ±TORQUE_FF_FIRMWARE_CLAMP_WIRE_NM, preserves sign, sets the mask") {
  const float LIM = Dynamics::TORQUE_FF_FIRMWARE_CLAMP_WIRE_NM;   // 0.25 wire-Nm
  float u0[6]    = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};

  SUBCASE("over-limit positive clamps to +LIM; in-bounds legs untouched; mask per leg") {
    reset_interp_test();
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.5f;   // encoder == command:
                                                              // no lead/stroke clamp
                                                              // (stroke zeroes torque)
    float tq[6] = {5.0f, 0.10f, 0.0f, 0.0f, 0.0f, 0.0f};      // leg0 absurd, leg1 normal
    stage(u0, nullptr, zeros, zeros, 0, tq);
    CHECK(s_pending);                                         // clamped, NOT rejected
    interp_isr();
    CHECK(axes[0].target_torque_Nm == doctest::Approx(LIM));  // bound, sign preserved
    CHECK(axes[1].target_torque_Nm == doctest::Approx(0.10f)); // untouched below the limit
    CHECK((interp_torque_clamp_mask() & 0x1u) != 0);          // leg0 flagged
    CHECK((interp_torque_clamp_mask() & 0x2u) == 0);          // leg1 not flagged
    CHECK(interp_torque_clamp_mask() == 0x1u);                // no other leg flagged
  }

  SUBCASE("over-limit negative clamps to -LIM (sign preserved)") {
    reset_interp_test();
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.5f;
    float tq[6] = {-1.0f, -0.10f, 0.0f, 0.0f, 0.0f, 0.0f};
    stage(u0, nullptr, zeros, zeros, 0, tq);
    CHECK(s_pending);
    interp_isr();
    CHECK(axes[0].target_torque_Nm == doctest::Approx(-LIM));
    CHECK(axes[1].target_torque_Nm == doctest::Approx(-0.10f));
    CHECK(interp_torque_clamp_mask() == 0x1u);
  }

  SUBCASE("at/below the threshold passes through untouched, mask stays clear") {
    reset_interp_test();
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.5f;
    float tq[6] = {LIM, -LIM, 0.1325f, -0.1325f, 0.0f, 0.0f};  // exactly at the bound +
                                                               // an in-range production-representative value (the pump clamp is 0.1451 wire-Nm since 2026-07-15)
    stage(u0, nullptr, zeros, zeros, 0, tq);
    interp_isr();
    CHECK(axes[0].target_torque_Nm == doctest::Approx(LIM));
    CHECK(axes[1].target_torque_Nm == doctest::Approx(-LIM));
    CHECK(axes[2].target_torque_Nm == doctest::Approx(0.1325f));
    CHECK(axes[3].target_torque_Nm == doctest::Approx(-0.1325f));
    CHECK(interp_torque_clamp_mask() == 0);
  }

  SUBCASE("mask CLEARS on the next accepted in-bounds frame") {
    reset_interp_test();
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.5f;
    float hot[6]  = {5.0f, 0, 0, 0, 0, 0};
    float cool[6] = {0.04f, 0, 0, 0, 0, 0};
    stage(u0, nullptr, zeros, zeros, 1, hot);
    interp_isr();
    CHECK(interp_torque_clamp_mask() == 0x1u);
    stage(u0, nullptr, zeros, zeros, 2, cool);
    interp_isr();
    CHECK(interp_torque_clamp_mask() == 0);                   // per-frame semantics
  }

  SUBCASE("a NaN torque_ff still drops the WHOLE frame (clamp does not sanitize NaN)") {
    reset_interp_test();
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.5f;
    float tq[6] = {std::nanf(""), 0, 0, 0, 0, 0};
    stage(u0, nullptr, zeros, zeros, 0, tq);
    CHECK_FALSE(s_pending);                                   // dropped before staging
    CHECK(interp_last_setpoint_us() == 0);                    // staleness clock NOT bumped
    CHECK(interp_torque_clamp_mask() == 0);                   // mask untouched by a drop
  }

  SUBCASE("a rejected (stale-seq) over-limit frame does NOT set the mask") {
    reset_interp_test();
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.5f;
    float cool[6] = {0.04f, 0, 0, 0, 0, 0};
    float hot[6]  = {5.0f, 0, 0, 0, 0, 0};
    stage(u0, nullptr, zeros, zeros, 10, cool);               // accepted, mask 0
    interp_isr();
    CHECK(interp_torque_clamp_mask() == 0);
    stage(u0, nullptr, zeros, zeros, 9, hot);                 // stale seq → dropped
    CHECK_FALSE(s_pending);
    CHECK(interp_torque_clamp_mask() == 0);                   // only ACCEPTED frames publish
  }
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

TEST_CASE("seq guard RE-BASELINES after a stream gap (host restart), review fix") {
  // The host resets its shared _tx_seq_stream to 0 each launch while the Jetson-5V
  // Teensy persists s_last_sp_seq — without the gap-reset, a LOW seq after a restart
  // would be dropped for minutes as 'stale' → a phantom MPC_STALE E-STOP.
  reset_interp_test();
  float a[6] = {0.10f, 0.10f, 0.10f, 0.10f, 0.10f, 0.10f};
  float c[6] = {0.30f, 0.30f, 0.30f, 0.30f, 0.30f, 0.30f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};

  // Establish a HIGH high-water (a long prior session), accepted.
  stage(a, nullptr, zeros, zeros, 20000);
  CHECK(s_pending);
  interp_isr();
  CHECK(interp_base_pos(0) == doctest::Approx(0.10f));

  // A gap longer than the staleness bound = the prior stream is dead (a restart).
  fake_advance(MPC_CMD_STALENESS_US + 1);

  // A LOW seq (host restarted its counter to ~0) must be ACCEPTED, not dropped —
  // (int16_t)(5 - 20000) = -19995 <= 0 would drop it WITHOUT the gap-reset.
  stage(c, nullptr, zeros, zeros, 5);
  CHECK(s_pending);                               // re-baselined, accepted
  interp_isr();
  CHECK(interp_base_pos(0) == doctest::Approx(0.30f));

  // And within the NEW session the guard is live again: a stale seq 4 drops.
  stage(a, nullptr, zeros, zeros, 4);
  CHECK_FALSE(s_pending);
}

// =============================================================================
//  2026-07-11 clear-errors jolt — the re-enable recovery slew
// =============================================================================
//  On the s_output_enabled false→true edge (a guard clear / arm) the ISR must
//  re-baseline the transmitted command to the LIVE ENCODER and SLEW toward the
//  streamed (lead-clamped) command with a bounded velocity+accel — never command the
//  diverged setpoint directly. Commanding it directly is what injected the
//  pos_gain × lead ≈ 4 rev/s kick to the −10 A current rail measured on both clear
//  events (forensics RESULT 3). These assert the transient is bounded, not a step,
//  and that a converged command is untouched.

TEST_CASE("re-enable recovery slew: a diverged command slews from the encoder (bounded, no step)") {
  reset_interp_test();
  axes[0].heartbeat_seen = true;                 // present
  axes[0].pos_rev = 1.0f;                        // live encoder = the leg's rest position
  // Command 0.15 rev BELOW the encoder → the lead clamp saturates: the streamed
  // target is encoder − MAX_LEAD_REV = 0.90. v0=accel=0 so cmd stays put across dt.
  float u0[6] = {0.85f, 0.85f, 0.85f, 0.85f, 0.85f, 0.85f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  const float target = 1.0f - MAX_LEAD_REV;      // 0.90 — the lead-clamped streamed command

  interp_set_output_enabled(false);
  stage(u0, nullptr, zeros, zeros);
  interp_isr();                                  // latch while disabled (no edge yet)
  CHECK_FALSE(s_recover_slewing);

  // ── The false→true edge ──
  interp_set_output_enabled(true);
  fake_advance(INTERP_PERIOD_US);
  interp_isr();
  CHECK(s_recover_slewing);
  // The first re-enabled frame is AT the encoder (dev≈0), NOT the −0.10 clamped
  // command — this is the whole fix: no pos_gain × lead velocity step.
  CHECK(axes[0].target_pos_rev == doctest::Approx(1.0f).epsilon(0.01));
  CHECK(std::fabs(axes[0].target_pos_rev - target) > 0.05f);   // decisively NOT the command
  CHECK(std::fabs(axes[0].target_vel_rps) < 0.1f);             // ~0 vel_ff at the edge

  // ── Drive the slew to convergence; bound the per-tick step + the vel_ff ──
  const float dt_tick = INTERP_PERIOD_US * 1e-6f;
  float prev = axes[0].target_pos_rev;
  float max_step = 0.0f, max_vel = 0.0f, min_pos = prev, max_pos = prev;
  int ticks = 0;
  while (s_recover_slewing && ticks < 2000) {
    fake_advance(INTERP_PERIOD_US);
    interp_isr();
    const float step = std::fabs(axes[0].target_pos_rev - prev);
    if (step > max_step) max_step = step;
    prev = axes[0].target_pos_rev;
    const float av = std::fabs(axes[0].target_vel_rps);
    if (av > max_vel) max_vel = av;
    if (axes[0].target_pos_rev < min_pos) min_pos = axes[0].target_pos_rev;
    if (axes[0].target_pos_rev > max_pos) max_pos = axes[0].target_pos_rev;
    ++ticks;
  }
  CHECK(max_vel <= RECOVER_SLEW_VEL_RPS + 1e-3f);              // velocity bounded (never the ~4 rev/s kick)
  CHECK(max_step <= RECOVER_SLEW_VEL_RPS * dt_tick + 1e-5f);   // no position step — bounded per tick
  CHECK(min_pos >= target - 1e-3f);              // never overshoots the command
  CHECK(max_pos <= 1.0f + 1e-3f);                // stays within the encoder+lead band

  // ── Handover: slew disarmed, normal streaming resumes on the clamped command ──
  CHECK_FALSE(s_recover_slewing);
  CHECK(axes[0].target_pos_rev == doctest::Approx(target).epsilon(0.01));
  fake_advance(INTERP_PERIOD_US);
  interp_isr();                                  // a normal (post-slew) tick
  CHECK_FALSE(s_recover_slewing);                // did NOT re-arm (no edge; output stayed enabled)
  CHECK(axes[0].target_pos_rev == doctest::Approx(target).epsilon(0.01));  // normal lead clamp holds
  CHECK(axes[0].target_vel_rps == doctest::Approx(0.0f).epsilon(0.02));    // normal vel_ff (v0=0)
}

TEST_CASE("re-enable recovery slew: a converged command is a one-tick no-op") {
  reset_interp_test();
  axes[0].heartbeat_seen = true;                 // present
  axes[0].pos_rev = 0.5f;                        // encoder
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};   // command already AT the encoder (the /recover happy path)
  float zeros[6] = {0, 0, 0, 0, 0, 0};

  interp_set_output_enabled(false);
  stage(u0, nullptr, zeros, zeros);
  interp_isr();                                  // latch while disabled

  interp_set_output_enabled(true);
  fake_advance(INTERP_PERIOD_US);
  fake_clear_sent();
  interp_isr();                                  // the edge tick
  // Converged: transmitted == command == encoder, and the slew disarms the SAME tick
  // (all present legs already within RECOVER_SLEW_DONE_EPS_REV) — normal streaming
  // is untouched except for this single benign edge tick.
  CHECK(axes[0].target_pos_rev == doctest::Approx(0.5f).epsilon(0.01));
  CHECK(std::fabs(axes[0].target_vel_rps) < 1e-3f);
  CHECK_FALSE(s_recover_slewing);
  CHECK(fake_sent_count_cmd(CMD_SETPOS) == 1);   // still streamed to the one present leg
}

// ── 2026-07-11 F2 fix: cold-start gate + lead re-clamp on the recovery slew ──────
//  A firmware home/activate/deactivate move drives the legs FASTER than the 1 rev/s
//  slew while the MPC leg TX is suppressed. Un-gated, the slew state would lag the
//  fast-moving encoder by more than MAX_LEAD and, un-re-clamped, emit an over-lead kick
//  at move-end. The fix (a) pins s_recover_pos to the LIVE encoder while any cold-start
//  move is active (clean edge, no stale advance), and (b) re-runs the lead clamp on the
//  slewed command so the EMITTED command can never exceed encoder±MAX_LEAD.

TEST_CASE("re-enable recovery slew: a cold-start move re-baselines the slew (never lags the encoder), F2") {
  reset_interp_test();
  axes[0].heartbeat_seen = true;                 // present
  axes[0].pos_rev = 1.0f;                        // encoder at rest
  // Command 0.15 rev below the encoder → lead-clamps to 0.90, so the slew has real work.
  float u0[6] = {0.85f, 0.85f, 0.85f, 0.85f, 0.85f, 0.85f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};

  interp_set_output_enabled(false);
  stage(u0, nullptr, zeros, zeros);
  interp_isr();                                  // latch while disabled
  interp_set_output_enabled(true);
  fake_advance(INTERP_PERIOD_US);
  interp_isr();                                  // edge → slew armed, baselined at 1.0
  REQUIRE(s_recover_slewing);

  // A cold-start move begins and sweeps the leg FAST (homing races the encoder ~1 rev
  // over 50 ticks = ~10 rev/s, an order of magnitude past the 1 rev/s slew).
  fake_set_homing(true);
  for (int k = 0; k < 50; ++k) {
    axes[0].pos_rev -= 0.02f;                    // encoder races down
    fake_advance(INTERP_PERIOD_US);
    interp_isr();
    // Fix (a): the slew state must track the LIVE encoder every tick, never lag it.
    CHECK(std::fabs(s_recover_pos[0] - axes[0].pos_rev) < 1e-4f);
  }
  fake_set_homing(false);
  // The move ended; a fresh command is latched near the new (low) encoder.
  float u0b[6]; for (int i = 0; i < 6; ++i) u0b[i] = axes[0].pos_rev;
  stage(u0b, nullptr, zeros, zeros, 1);
  fake_advance(INTERP_PERIOD_US);
  interp_isr();
  // The emitted command can NEVER exceed encoder±MAX_LEAD — the bug was an over-lead
  // kick here (s_recover_pos stranded ~1 rev above the drifted encoder).
  CHECK(std::fabs(axes[0].target_pos_rev - axes[0].pos_rev) <= MAX_LEAD_REV + 1e-4f);
}

TEST_CASE("re-enable recovery slew: the emitted command is always re-clamped to encoder±MAX_LEAD, F2") {
  reset_interp_test();
  axes[0].heartbeat_seen = true;                 // present
  axes[0].pos_rev = 0.5f;                        // encoder
  // Diverged command (0.15 rev below) → lead-clamps to 0.40, so the slew stays ACTIVE
  // for several ticks (a converged command would disarm on the edge tick).
  float u0[6] = {0.35f, 0.35f, 0.35f, 0.35f, 0.35f, 0.35f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};

  interp_set_output_enabled(false);
  stage(u0, nullptr, zeros, zeros);
  interp_isr();                                  // latch while disabled
  interp_set_output_enabled(true);
  fake_advance(INTERP_PERIOD_US);
  interp_isr();                                  // edge → slew armed at 0.5, still slewing
  REQUIRE(s_recover_slewing);

  // Force the slew state to lag the encoder by FAR more than the lead clamp (the
  // pathological lag the cold-start gate prevents) and prove fix (b) bounds BOTH the
  // emitted command AND the slew state on the very next tick.
  s_recover_pos[0] = 0.5f + 5.0f;                // 5 rev past the encoder — absurd, on purpose
  fake_advance(INTERP_PERIOD_US);
  interp_isr();
  CHECK(std::fabs(axes[0].target_pos_rev - axes[0].pos_rev) <= MAX_LEAD_REV + 1e-4f);
  // And the slew STATE is pulled back inside the band too (never keeps running away).
  CHECK(std::fabs(s_recover_pos[0] - axes[0].pos_rev) <= MAX_LEAD_REV + 1e-4f);
}
