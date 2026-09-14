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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

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

// ── FW 17 v6 helpers — the hand lane ─────────────────────────────────────────
struct HandKnots { float u0, u1, u2, v0, accel, v1; };

// Stage a full v6 frame: legs flat + an optional hand lane + optional exact v1.
// hk != nullptr sets HAS_HAND (index 6 live); v1legs != nullptr sets HAS_V1 and
// carries the leg v1 array (hk->v1 rides index 6). has_u2 fills u2 = u1 + (u1-u0)
// per leg so the (u2-u1)/SEG_T fallback is a defined quantity.
static void stage_hand(const float u0[6], const float* u1, const float v0[6],
                       const HandKnots* hk, uint16_t seq,
                       const float* v1legs = nullptr, bool has_u2 = false) {
  JbUdp::SetpointPayload sp;
  memset(&sp, 0, sizeof(sp));
  for (int i = 0; i < 6; ++i) {
    sp.u0[i] = u0[i];
    if (v0) sp.v0[i] = v0[i];
    if (u1) {
      sp.u1[i] = u1[i];
      if (has_u2) sp.u2[i] = u1[i] + (u1[i] - u0[i]);
    }
    if (v1legs) sp.v1[i] = v1legs[i];
  }
  sp.flags = (u1 ? 0x1u : 0x0u) | ((u1 && has_u2) ? 0x2u : 0x0u);
  if (hk) {
    sp.u0[6] = hk->u0; sp.u1[6] = hk->u1; sp.u2[6] = hk->u2;
    sp.v0[6] = hk->v0; sp.accel[6] = hk->accel; sp.v1[6] = hk->v1;
    sp.flags |= 0x4u;                    // HAS_HAND
  }
  if (v1legs) sp.flags |= 0x8u;          // HAS_V1
  interp_on_setpoint(seq, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
}

// Park axis 6 at the retract rest on FRESH telemetry — the I-HAND-5 precondition
// (the lane never transmits before the first axis-6 encoder frame). Since FW 21
// there is no mastery latch to flip: a HAS_HAND frame is the whole arming story,
// so this seeds the encoder and nothing else. Tests then move the fake encoder
// wherever the case needs it.
static void seed_hand_encoder() {
  hand_axis().heartbeat_seen = true;
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
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
//  degrade to a bounded torque, never starve the interp into an SETPOINT_STALE
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
  // frames must still eventually trip SETPOINT_STALE (as if the link went quiet).
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
  // would be dropped for minutes as 'stale' → a phantom SETPOINT_STALE E-STOP.
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
  fake_advance(SETPOINT_STALENESS_US + 1);

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

// ═══════════════════════════════════════════════════════════════════════════
//  TRI-STATE TX (2026-08-24) — the 500 Hz setpoint burst's owner-delegated ruling
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("a DEFERRED leg setpoint is SENT: counted, charged to LEGS, never retried") {
  reset_interp_test();
  for (uint8_t i = 0; i < NUM_LEGS; ++i) axes[i].heartbeat_seen = true;
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  interp_set_output_enabled(true);
  stage(u0, nullptr, zeros, zeros);

  // A saturated mailbox set: every setpoint in the burst defers into the software
  // txBuffer. THE RULING: that is SENT. It transmits, in order, ~0.1-1 ms later.
  fake_clear_sent();
  fake_set_send_defer_all(true);
  interp_isr();

  // Exactly ONE frame per present leg. NOT ONE MORE. The retry that a "failed"
  // reading would invite is the wrong move here and always was: these are
  // latest-wins setpoints at 500 Hz, so re-sending a frame the queue already holds
  // puts a STALE setpoint on the wire behind a fresher one — the interp ladder's
  // whole contract is that the newest command wins.
  CHECK(fake_sent_count_cmd(CMD_SETPOS) == (size_t)NUM_LEGS);
  CHECK(fake_sent_count() == (size_t)NUM_LEGS);

  // Charged to the LEGS bucket, so leg-burst pressure can never be mistaken for a
  // deferred safety frame in the census.
  CHECK(fake_sent_count_cls(TxCls::LEGS) == (size_t)NUM_LEGS);
  CHECK(fake_sent_count_cls(TxCls::SAFETY) == 0u);

  // And the next tick behaves identically — no backlog, no accumulated retry queue,
  // no fault-machine involvement. The ISR does not even look at the result.
  fake_clear_sent();
  fake_advance(INTERP_PERIOD_US);
  interp_isr();
  CHECK(fake_sent_count() == (size_t)NUM_LEGS);
}


// ═══════════════════════════════════════════════════════════════════════════
//  FW 17 — the hand lane (unified-7dof Phase 3, T-U9)
// ═══════════════════════════════════════════════════════════════════════════
//  The hand lane's parity anchor is the IDENTICAL-KNOTS rule: for the same
//  knots, the hand block must produce bit-for-bit the leg block's output in
//  every ladder mode. motor_guard.py is 6-lane BY CONTRACT and is deliberately
//  not modified (the xref chain is the trust anchor) — identical-lane parity
//  TRANSFERS the leg xref's trust to the shared math, which is exactly how the
//  hand lane earns its ladder without a 7-lane Python reference.

TEST_CASE("hand lane inert when HAS_HAND clear (v6 frame, index 6 ignored)") {
  reset_interp_test();
  seed_hand_encoder();                            // encoder fresh: still no HAS_HAND ⇒ no lane
  axes[HAND_AXIS].heartbeat_seen = true;
  interp_set_output_enabled(true);
  axes[0].heartbeat_seen = true;
  axes[0].pos_rev = 0.5f;
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  stage(u0, nullptr, zeros, zeros, 1);            // 6-lane staging: flags carry no HAS_HAND
  hand_axis().target_pos_rev = -1.0f;             // sentinel: must stay untouched
  fake_clear_sent();
  interp_isr();
  CHECK_FALSE(interp_hand_lane_active());
  CHECK(hand_axis().target_pos_rev == doctest::Approx(-1.0f));
  bool hand_streamed_frame = false;
  for (size_t i = 0; i < fake_sent_count(); ++i)
    if (ODrive::axis_of(fake_sent_at(i).id) == HAND_AXIS) hand_streamed_frame = true;
  CHECK_FALSE(hand_streamed_frame);               // no 7th frame
  CHECK(interp_hand_sent() == 0);
}

// FW 21 (skill-stack R1): there is no mastery latch left to consult. A HAS_HAND
// frame latches the lane on its own — the old "discard index 6 while LEGACY"
// branch and its s_hand_discard_legacy counter are gone, so the SAME frame that
// used to be refused by mode is now simply accepted. This case is the fence
// against a second gate creeping back in front of the lane.
TEST_CASE("a HAS_HAND frame latches the lane with no latch to consult") {
  reset_interp_test();
  axes[0].heartbeat_seen = true;
  seed_hand_encoder();
  write_pos_vel(hand_axis(), 1.0f, 0.0f, fake_mono_us());
  interp_set_output_enabled(true);
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  axes[0].pos_rev = 0.5f;
  HandKnots hk{1.0f, 1.1f, 0.0f, 0.5f, 0.0f, 0.0f};
  fake_clear_sent();
  stage_hand(u0, nullptr, zeros, &hk, 1);
  interp_isr();
  CHECK(interp_hand_lane_active());                     // latched by HAS_HAND alone
  CHECK(interp_base_pos(0) == doctest::Approx(0.5f));   // the legs still latched
  bool hand_frame = false;
  for (size_t i = 0; i < fake_sent_count(); ++i)
    if (ODrive::axis_of(fake_sent_at(i).id) == HAND_AXIS) hand_frame = true;
  CHECK(hand_frame);                                    // and the 7th frame goes out
  CHECK(interp_hand_sent() == 1);
}

TEST_CASE("7-lane Hermite parity: identical knots ⇒ hand output == leg output, bit for bit") {
  // Mode 1 with the TRANSMITTED v1 (HAS_V1), sampled across the segment. Values
  // chosen to keep BOTH lanes clamp-free (leg vel cap 3.5, leg lead 0.10) so
  // the comparison is of the LADDER, not of the per-lane clamps.
  reset_interp_test();
  seed_hand_encoder();
  const uint64_t t_now = fake_mono_us();
  write_pos_vel(hand_axis(), 0.52f, 0.0f, t_now);
  for (uint8_t i = 0; i < 6; ++i) { axes[i].pos_rev = 0.52f; axes[i].heartbeat_seen = true; }
  float u0[6] = {0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f};
  float u1[6] = {0.55f, 0.55f, 0.55f, 0.55f, 0.55f, 0.55f};
  float v0[6] = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
  float v1l[6] = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
  HandKnots hk{0.50f, 0.55f, 0.0f, 2.0f, 0.0f, 2.0f};
  stage_hand(u0, u1, v0, &hk, 1, v1l);
  for (int k = 0; k < 12; ++k) {                  // 12 ticks × 2 ms spans the 25 ms segment
    interp_isr();
    CHECK(interp_hand_lane_active());
    CHECK(axes[HAND_AXIS].target_pos_rev == axes[0].target_pos_rev);
    CHECK(axes[HAND_AXIS].target_vel_rps == axes[0].target_vel_rps);
    fake_advance(INTERP_PERIOD_US);
  }
}

TEST_CASE("7-lane parity holds in Taylor extrapolation and velocity decay (Modes 2/3)") {
  reset_interp_test();
  seed_hand_encoder();
  write_pos_vel(hand_axis(), 0.5f, 0.0f, fake_mono_us());
  for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.5f;
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float v0[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  HandKnots hk{0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
  stage_hand(u0, nullptr, v0, &hk, 1);            // no u1 ⇒ Mode 2/3 on every lane
  interp_isr();                                   // latch at dt≈0
  fake_advance(20'000);                           // dt = 0.02 ≤ MAX_EXTRAP (Mode 2)
  interp_isr();
  CHECK(axes[HAND_AXIS].target_pos_rev == axes[0].target_pos_rev);
  CHECK(axes[HAND_AXIS].target_vel_rps == axes[0].target_vel_rps);
  fake_advance((uint64_t)((MAX_EXTRAP_DT_S + EXTRAP_DECAY_DT_S) * 1e6f));   // deep into Mode 3
  interp_isr();
  CHECK(axes[HAND_AXIS].target_pos_rev == axes[0].target_pos_rev);
  CHECK(axes[HAND_AXIS].target_vel_rps == axes[0].target_vel_rps);
  CHECK(std::fabs(axes[HAND_AXIS].target_vel_rps) < 0.05f);   // both decayed to rest
}

TEST_CASE("Mode-1 endpoint velocity rules pinned: transmitted v1 exact; (u2-u1)/T fallback; (u1-u0)/T last") {
  // At s = 1 the Hermite velocity IS the endpoint v1 (dh00=dh01=dh10=0, dh11=1),
  // so sampling the segment end reads the v1 rule directly, on both lanes.
  float zeros[6] = {0, 0, 0, 0, 0, 0};

  SUBCASE("HAS_V1: the TRANSMITTED value, exactly — not a difference of knots") {
    reset_interp_test();
    seed_hand_encoder();
    write_pos_vel(hand_axis(), 0.52f, 0.0f, fake_mono_us());
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.52f;
    float u0[6] = {0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f};
    float u1[6] = {0.55f, 0.55f, 0.55f, 0.55f, 0.55f, 0.55f};
    float v0[6] = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
    float v1l[6] = {3.25f, 3.25f, 3.25f, 3.25f, 3.25f, 3.25f};   // ≠ (u1-u0)/T = 2.0
    HandKnots hk{0.50f, 0.55f, 0.0f, 2.0f, 0.0f, 3.25f};
    stage_hand(u0, u1, v0, &hk, 1, v1l);
    interp_isr();                                  // latch
    fake_advance((uint64_t)(SEGMENT_T_S * 1e6f));  // s = 1
    interp_isr();
    CHECK(axes[0].target_vel_rps == doctest::Approx(3.25f).epsilon(1e-5));
    CHECK(axes[HAND_AXIS].target_vel_rps == doctest::Approx(3.25f).epsilon(1e-5));
  }

  SUBCASE("HAS_V1 clear + u2 present: the flown (u2-u1)/SEG_T forward difference") {
    reset_interp_test();
    seed_hand_encoder();
    write_pos_vel(hand_axis(), 0.52f, 0.0f, fake_mono_us());
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.52f;
    JbUdp::SetpointPayload sp;
    memset(&sp, 0, sizeof(sp));
    for (int i = 0; i < 7; ++i) {
      sp.u0[i] = 0.50f; sp.u1[i] = 0.55f;
      sp.u2[i] = 0.62f;                            // (u2-u1)/T = 0.07/0.025 = 2.8
      sp.v0[i] = 2.0f;
    }
    sp.flags = 0x1u | 0x2u | 0x4u;                 // HAS_U1 | HAS_U2 | HAS_HAND, no HAS_V1
    interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
    fake_advance((uint64_t)(SEGMENT_T_S * 1e6f));
    interp_isr();
    CHECK(axes[0].target_vel_rps == doctest::Approx(2.8f).epsilon(1e-4));
    CHECK(axes[HAND_AXIS].target_vel_rps == doctest::Approx(2.8f).epsilon(1e-4));
  }

  SUBCASE("HAS_V1 clear, no u2: the (u1-u0)/SEG_T last resort") {
    reset_interp_test();
    seed_hand_encoder();
    write_pos_vel(hand_axis(), 0.52f, 0.0f, fake_mono_us());
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.52f;
    float u0[6] = {0.50f, 0.50f, 0.50f, 0.50f, 0.50f, 0.50f};
    float u1[6] = {0.55f, 0.55f, 0.55f, 0.55f, 0.55f, 0.55f};
    float v0[6] = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
    HandKnots hk{0.50f, 0.55f, 0.0f, 2.0f, 0.0f, 0.0f};
    stage_hand(u0, u1, v0, &hk, 1);
    interp_isr();
    fake_advance((uint64_t)(SEGMENT_T_S * 1e6f));
    interp_isr();
    CHECK(axes[0].target_vel_rps == doctest::Approx(2.0f).epsilon(1e-4));   // (0.55-0.50)/0.025
    CHECK(axes[HAND_AXIS].target_vel_rps == doctest::Approx(2.0f).epsilon(1e-4));
  }
}

TEST_CASE("NORMATIVE falling edge: HAS_HAND falls while leg frames continue ⇒ the hand DECAYS, never holds v1") {
  // The Phase 2 review carry-in, pinned. Frame 1 carries a hand segment ending
  // at v1 = 10 rev/s; every later frame is hand-less while the LEG stream stays
  // fresh. Without the per-lane knot clock the hand would sit at Mode 1's s = 1
  // forever, commanding the endpoint with vel_ff = v1 — hold-at-last-command.
  // Required: segment completes, extrapolates ≤ MAX_EXTRAP, then vel → 0.
  reset_interp_test();
  seed_hand_encoder();
  write_pos_vel(hand_axis(), 5.0f, 0.0f, fake_mono_us());
  for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.5f;
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float u1[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  float v1l[6] = {0, 0, 0, 0, 0, 0};
  HandKnots hk{5.0f, 5.1f, 0.0f, 4.0f, 0.0f, 10.0f};
  stage_hand(u0, u1, zeros, &hk, 1, v1l);
  interp_isr();                                   // latch the hand-bearing frame
  CHECK(interp_hand_lane_active());

  // The falling edge: a fresh LEG frame with no hand channel.
  fake_advance(INTERP_PERIOD_US);
  stage(u0, u1, zeros, zeros, 2);
  interp_isr();
  CHECK(interp_hand_lane_active());               // lane still holds its own knot

  // Just past the hand segment end: still moving (the segment completed and the
  // endpoint state carries it into extrapolation — NOT frozen mid-segment).
  fake_advance((uint64_t)(SEGMENT_T_S * 1e6f));
  interp_isr();
  CHECK(std::fabs(axes[HAND_AXIS].target_vel_rps) > 1.0f);

  // Deep past MAX_EXTRAP + DECAY (measured from the hand's own knot): decayed.
  fake_advance((uint64_t)((MAX_EXTRAP_DT_S + EXTRAP_DECAY_DT_S + 0.01f) * 1e6f));
  interp_isr();
  CHECK(std::fabs(axes[HAND_AXIS].target_vel_rps) < 0.01f);   // vel_ff decayed to zero
  CHECK(std::fabs(axes[HAND_AXIS].target_vel_rps - 10.0f) > 5.0f);  // decisively NOT the held v1
  const float settled = axes[HAND_AXIS].target_pos_rev;
  // ~5.1 (endpoint) + 10·0.05 (extrap) + 10·DECAY/2 (decay distance) = 5.9
  CHECK(settled == doctest::Approx(5.9f).epsilon(0.02));
  fake_advance(INTERP_PERIOD_US);
  interp_isr();
  CHECK(axes[HAND_AXIS].target_pos_rev == doctest::Approx(settled));  // at rest, holding
}

TEST_CASE("guard separation: the LEG constants never touch axis 6") {
  SUBCASE("lead clamp: 1.5 rev of hand deviation passes (band 2.0), the same leg deviation clamps at 0.10") {
    reset_interp_test();
    seed_hand_encoder();
    write_pos_vel(hand_axis(), 1.0f, 0.0f, fake_mono_us());
    axes[0].pos_rev = 1.0f; axes[0].heartbeat_seen = true;
    float u0[6] = {2.5f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f};   // leg 0: 1.5 rev ahead
    float zeros[6] = {0, 0, 0, 0, 0, 0};
    HandKnots hk{2.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};          // hand: same 1.5 rev ahead
    stage_hand(u0, nullptr, zeros, &hk, 1);
    interp_isr();
    CHECK(axes[0].target_pos_rev == doctest::Approx(1.0f + MAX_LEAD_REV));       // leg clamped at 0.10
    CHECK(axes[HAND_AXIS].target_pos_rev == doctest::Approx(2.5f));              // hand passes
    CHECK((interp_lead_clamp_mask() & 0x01u) != 0);
    CHECK((interp_lead_clamp_mask() & 0x40u) == 0);            // hand bit clear
    CHECK(interp_hand_lead_clamp_ticks() == 0);
  }

  SUBCASE("hand lead clamp binds at MAX_LEAD_HAND_REV, sets mask bit 6 + the lead-duty counter") {
    reset_interp_test();
    seed_hand_encoder();
    write_pos_vel(hand_axis(), 1.0f, 0.0f, fake_mono_us());
    float u0[6] = {0.07f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f};
    float zeros[6] = {0, 0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.07f;
    HandKnots hk{4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};          // 3.0 rev ahead of fb 1.0
    stage_hand(u0, nullptr, zeros, &hk, 1);
    interp_isr();
    CHECK(axes[HAND_AXIS].target_pos_rev == doctest::Approx(1.0f + MAX_LEAD_HAND_REV));
    CHECK((interp_lead_clamp_mask() & 0x40u) != 0);
    // FW 18: output is SUPPRESSED here (this subcase never arms it), so the
    // clamp binds, the mask bit sets and the residual is observed — but the
    // CUMULATIVE counters stay at zero, because nothing reached the wire. The
    // transmitting case is the "a TRANSMITTING stage does count…" test below.
    CHECK(interp_hand_lead_clamp_ticks() == 0);
    CHECK(interp_hand_dev_over_ticks() == 0);
    // The deviation verdict still saw the RAW 3.0 rev residual (> 2.5).
    CHECK(interp_hand_dev_max() == doctest::Approx(3.0f).epsilon(0.01));
  }

  SUBCASE("vel_ff cap is 300 (HAND_VELFF_LIMIT_RPS), not the legs' 3.5") {
    reset_interp_test();
    seed_hand_encoder();
    write_pos_vel(hand_axis(), 1.0f, 0.0f, fake_mono_us());
    float u0[6] = {0.07f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f};
    float zeros[6] = {0, 0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.07f;
    SUBCASE("50 rev/s passes untouched (the legs' cap would crush it 14×)") {
      HandKnots hk{1.0f, 0.0f, 0.0f, 50.0f, 0.0f, 0.0f};
      stage_hand(u0, nullptr, zeros, &hk, 1);
      interp_isr();
      CHECK(axes[HAND_AXIS].target_vel_rps == doctest::Approx(50.0f));
    }
    SUBCASE("350 rev/s is bounded to 300") {
      HandKnots hk{1.0f, 0.0f, 0.0f, 350.0f, 0.0f, 0.0f};
      stage_hand(u0, nullptr, zeros, &hk, 1);
      interp_isr();
      CHECK(axes[HAND_AXIS].target_vel_rps == doctest::Approx(HAND_VELFF_LIMIT_RPS));
    }
  }

  SUBCASE("hand stroke clip is [0, HAND_MOTOR_MAX_POSITION] (10.501), never the leg stroke table") {
    reset_interp_test();
    seed_hand_encoder();
    write_pos_vel(hand_axis(), 10.5f, 0.0f, fake_mono_us());
    float u0[6] = {0.07f, 0.07f, 0.07f, 0.07f, 0.07f, 0.07f};
    float zeros[6] = {0, 0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < 6; ++i) axes[i].pos_rev = 0.07f;
    HandKnots hk{12.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f};         // past the metal
    stage_hand(u0, nullptr, zeros, &hk, 1);
    interp_isr();
    CHECK(axes[HAND_AXIS].target_pos_rev == doctest::Approx(HAND_MOTOR_MAX_POSITION));
    CHECK(axes[HAND_AXIS].target_vel_rps == doctest::Approx(0.0f));   // FF zeroed at the stop
  }
}

TEST_CASE("hand lane never transmits before the first axis-6 encoder frame (unseen skip, counted)") {
  reset_interp_test();
  seed_hand_encoder();
  axes[0].heartbeat_seen = true;
  axes[0].pos_rev = 0.5f;
  interp_set_output_enabled(true);
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  HandKnots hk{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  stage_hand(u0, nullptr, zeros, &hk, 1);
  hand_axis().pos_timestamp_us = 0;               // never seen (the switch's fresh sample expired conceptually)
  fake_clear_sent();
  interp_isr();
  CHECK(interp_hand_unseen_skips() == 1);
  bool hand_streamed_frame = false;
  for (size_t i = 0; i < fake_sent_count(); ++i)
    if (ODrive::axis_of(fake_sent_at(i).id) == HAND_AXIS) hand_streamed_frame = true;
  CHECK_FALSE(hand_streamed_frame);
  CHECK(fake_sent_count_cmd(CMD_SETPOS) == 1);    // the leg still streamed
}

TEST_CASE("the 7th frame rides the burst: axis-6 set_input_pos, byte-exact, zero torque, TxCls::LEGS") {
  reset_interp_test();
  seed_hand_encoder();
  write_pos_vel(hand_axis(), 1.0f, 0.0f, fake_mono_us());
  axes[HAND_AXIS].heartbeat_seen = true;
  axes[0].heartbeat_seen = true;
  axes[0].pos_rev = 0.5f;
  interp_set_output_enabled(true);
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  HandKnots hk{1.0f, 1.02f, 0.0f, 0.8f, 0.0f, 0.8f};
  float v1l[6] = {0, 0, 0, 0, 0, 0};
  stage_hand(u0, u0, zeros, &hk, 1, v1l);   // legs hold at u0 (u1 == u0)
  fake_clear_sent();
  interp_isr();
  CHECK(interp_hand_sent() == 1);
  size_t hand_i = SIZE_MAX;
  for (size_t i = 0; i < fake_sent_count(); ++i)
    if (ODrive::axis_of(fake_sent_at(i).id) == HAND_AXIS) hand_i = i;
  REQUIRE(hand_i != SIZE_MAX);
  CHECK(fake_sent_at(hand_i).cls == TxCls::LEGS);   // the 7th interpolated axis, not a hand_ops dispatch
  const auto expect = ODrive::encode_leg_setpoint(
      HAND_AXIS, axes[HAND_AXIS].target_pos_rev, axes[HAND_AXIS].target_vel_rps, 0.0f);
  CHECK(fake_sent_at(hand_i).id == expect.id);
  CHECK(memcmp(fake_sent_at(hand_i).buf, expect.buf, 8) == 0);
  // torque bytes (int16 at [6..7]) are hard zero on the hand lane in FW 17.
  CHECK(fake_sent_at(hand_i).buf[6] == 0);
  CHECK(fake_sent_at(hand_i).buf[7] == 0);
}


// (A test named "a latched hand lane goes INERT the tick the latch returns to
// LEGACY" lived here until FW 21. It asserted the mastery latch's own
// dual-master fence; with the latch and the Platform stroke engine deleted
// there is no second master and no second gate, so the case has no subject.
// The falling-edge decay, the arm-edge latch clear and the unseen-skip — the
// rules that actually bound a stale hand lane — are asserted above and below.)

// ═══════════════════════════════════════════════════════════════════════════
//  2026-09-02 adversarial-review fixes — hand-lane session hygiene + slew split
// ═══════════════════════════════════════════════════════════════════════════
//  interp_reset() has no runtime caller, so the hand-lane latch used to persist
//  across armed SESSIONS: a later hand-less armed session found the ancient
//  latch alive and replayed its decayed
//  hold as a live 7th frame. The fix clears the lane on the s_output_enabled
//  false→true edge (before the staging consume, so a live stream's pending
//  HAS_HAND frame re-latches the same tick) — a fresh HAS_HAND latch per armed
//  session, by construction.

TEST_CASE("arm-edge clears a stale hand latch: a hand-less re-armed session is INERT (2026-09-02 fix)") {
  reset_interp_test();
  seed_hand_encoder();
  write_pos_vel(hand_axis(), 1.0f, 0.0f, fake_mono_us());
  axes[HAND_AXIS].heartbeat_seen = true;
  axes[0].heartbeat_seen = true; axes[0].pos_rev = 0.5f;
  interp_set_output_enabled(true);
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  HandKnots hk{1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  stage_hand(u0, u0, zeros, &hk, 1);        // armed session 1: hand-bearing
  interp_isr();                             // edge clear, then the fresh frame re-latches
  CHECK(interp_hand_lane_active());
  CHECK(interp_hand_sent() == 1);

  // Session 1 ends: disarm (output gated off).
  interp_set_output_enabled(false);
  fake_advance(INTERP_PERIOD_US);
  interp_isr();

  // Session 2: re-arm HAND-LESS — the (c2) node
  // fold runs the hand preamble on the latch alone, no hand knots needed, so
  // this is exactly the stale-replay window. The lane must stay inert.
  hand_axis().target_pos_rev = -3.0f;       // sentinel: must stay untouched
  interp_set_output_enabled(true);
  fake_clear_sent();
  uint16_t seq = 2;
  for (int k = 0; k < 5; ++k) {
    fake_advance(INTERP_PERIOD_US);
    stage(u0, nullptr, zeros, zeros, seq++);   // fresh LEG frames, no hand channel
    interp_isr();
  }
  CHECK_FALSE(interp_hand_lane_active());
  CHECK(interp_hand_sent() == 1);           // no 7th frame in session 2
  bool hand_streamed_frame = false;
  for (size_t i = 0; i < fake_sent_count(); ++i)
    if (ODrive::axis_of(fake_sent_at(i).id) == HAND_AXIS) hand_streamed_frame = true;
  CHECK_FALSE(hand_streamed_frame);
  CHECK(hand_axis().target_pos_rev == doctest::Approx(-3.0f));   // untouched
  CHECK(fake_sent_count_cmd(CMD_SETPOS) == 5u);   // the leg stream ran all five ticks
}

TEST_CASE("stale latch + LEGACY→STREAMED round trip: still inert after re-arm until a FRESH HAS_HAND knot (2026-09-02 fix)") {
  reset_interp_test();
  seed_hand_encoder();                      // hand settled at the 0.0 rest
  axes[HAND_AXIS].heartbeat_seen = true;
  axes[0].heartbeat_seen = true; axes[0].pos_rev = 0.5f;
  interp_set_output_enabled(true);
  float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  float zeros[6] = {0, 0, 0, 0, 0, 0};
  HandKnots hk{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // hold at the rest position
  stage_hand(u0, u0, zeros, &hk, 1);
  interp_isr();
  CHECK(interp_hand_sent() == 1);

  // Disarm — the stale knot state rides across the disarm/re-arm boundary.
  interp_set_output_enabled(false);
  fake_advance(INTERP_PERIOD_US);
  interp_isr();
  seed_hand_encoder();                      // encoder still fresh

  // Re-arm: the stale session-1 latch must NOT go live again.
  interp_set_output_enabled(true);
  fake_clear_sent();
  fake_advance(INTERP_PERIOD_US);
  stage(u0, nullptr, zeros, zeros, 2);      // hand-less frame
  interp_isr();                             // the arm edge — stale latch cleared
  CHECK_FALSE(interp_hand_lane_active());
  CHECK(interp_hand_sent() == 1);           // still inert …

  // … until a FRESH HAS_HAND knot latches the lane for THIS session.
  fake_advance(INTERP_PERIOD_US);
  stage_hand(u0, nullptr, zeros, &hk, 3);
  interp_isr();
  CHECK(interp_hand_lane_active());
  CHECK(interp_hand_sent() == 2);
}

TEST_CASE("per-axis-group recovery slew: legs resume full FF while the hand still slews (2026-09-02 fix)") {
  // Pre-fix the hand joined the recovery slew under the legs' shared all_done:
  // every output-enable edge held ALL SIX LEGS at vel_ff = torque_ff = 0 for
  // the up-to-~2 s a 2.0 rev hand excursion takes at the 1 rev/s slew speed
  // (was ~0.1 s legs-only). Post-fix the leg set hands back on its OWN
  // convergence; the hand converges on its own flag, clock and speed ramp.
  reset_interp_test();
  seed_hand_encoder();
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
  axes[HAND_AXIS].heartbeat_seen = true;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) { axes[i].heartbeat_seen = true; axes[i].pos_rev = 0.5f; }
  interp_set_output_enabled(true);
  JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
  for (int i = 0; i < 6; ++i) { sp.u0[i] = 0.5f; sp.torque_ff[i] = 0.1f; }   // legs AT their encoders, real FF
  sp.u0[HAND_AXIS] = 1.5f;                  // hand commanded 1.5 rev away (inside the 2.0 lead band)
  sp.flags = 0x4u;                          // HAS_HAND, no u1 (Mode-2/3 hold)
  interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));

  // The output-enable edge tick: the pending frame re-latches after the edge
  // clear; the LEG set is already converged (cmd == encoder) so the leg slew
  // disarms this same tick, while the hand starts its own bounded ramp.
  interp_isr();
  CHECK_FALSE(s_recover_slewing);
  CHECK(s_hand_recover_slewing);

  // Next tick: legs stream NORMAL commands — torque FF restored — while the
  // hand is still slewing toward its 1.5 rev command.
  fake_advance(INTERP_PERIOD_US);
  interp_isr();
  CHECK(s_hand_recover_slewing);
  CHECK(axes[0].target_torque_Nm == doctest::Approx(0.1f));   // THE fix: FF back while the hand slews
  CHECK(axes[HAND_AXIS].target_pos_rev < 0.5f);               // hand still far from 1.5

  // Drive the hand to convergence on its own clock; the legs keep full FF the
  // whole way and the hand's slew velocity stays bounded.
  float max_hand_vel = 0.0f;
  bool legs_ff_held = true;
  int ticks = 0;
  while (s_hand_recover_slewing && ticks < 2000) {
    fake_advance(INTERP_PERIOD_US);
    interp_isr();
    const float av = std::fabs(axes[HAND_AXIS].target_vel_rps);
    if (av > max_hand_vel) max_hand_vel = av;
    if (std::fabs(axes[0].target_torque_Nm - 0.1f) > 1e-4f) legs_ff_held = false;
    ++ticks;
  }
  CHECK_FALSE(s_hand_recover_slewing);
  CHECK(ticks < 2000);
  CHECK(legs_ff_held);
  CHECK(max_hand_vel <= RECOVER_SLEW_VEL_RPS + 1e-3f);
  CHECK(axes[HAND_AXIS].target_pos_rev == doctest::Approx(1.5f).epsilon(0.01));
}

// ═══ FW 18: the hand counter gate + `hand7 reset` ════════════════════════════
//
//  The [hand7] lead / dev_over counters are cumulative and boot-zero, and the
//  runbook reads them as ABSOLUTES ("non-zero lead during a throw ⇒ hard-abort
//  the sitting"). Until FW 18 they counted every tick the lane computed, whether
//  or not the resulting frame reached the wire — so an aborted stage (an E-STOP
//  latch, a cold-start move, a bench stage stopped early) left them non-zero
//  forever and the abort rule was unfalsifiable: every reading had to be
//  differenced across the stage by hand. Gated on the SAME
//  `s_output_enabled && !coldstart` condition as the TX, a suppressed lane
//  counts nothing and an absolute read means something again.

// Park the encoder at 0 and command the hand far away, so the deviation exceeds
// MAX_DEVIATION_HAND_REV (2.5) AND the lead clamp exceeds MAX_LEAD_HAND_REV
// (2.0) on every tick the lane computes.
static void stage_hand_far_from_encoder(uint16_t seq) {
  static const float u0[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  static const float zeros[6] = {0, 0, 0, 0, 0, 0};
  HandKnots hk{6.0f, 6.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // 6 rev away from fb = 0
  stage_hand(u0, u0, zeros, &hk, seq);
}

TEST_CASE("an ABORTED (output-suppressed) stage leaves the hand counters at ZERO") {
  reset_interp_test();
  seed_hand_encoder();
  axes[HAND_AXIS].heartbeat_seen = true;
  axes[0].heartbeat_seen = true; axes[0].pos_rev = 0.5f;
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());

  interp_set_output_enabled(false);          // the stage aborted / guard latched
  uint16_t seq = 1;
  for (int k = 0; k < 20; ++k) {
    fake_advance(INTERP_PERIOD_US);
    write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());   // keep fb fresh
    stage_hand_far_from_encoder(seq++);
    interp_isr();
  }
  // The lane really did compute (this is not a vacuous pass): the residual is
  // observed and huge — it is only the CUMULATIVE counters that are gated.
  CHECK(interp_hand_lane_active());
  CHECK(interp_hand_dev_last() > MAX_DEVIATION_HAND_REV);
  CHECK(interp_hand_lead_clamp_ticks() == 0u);
  CHECK(interp_hand_dev_over_ticks() == 0u);
  // …and nothing reached the wire, which is what the counters now mean.
  bool hand_frame = false;
  for (size_t i = 0; i < fake_sent_count(); ++i)
    if (ODrive::axis_of(fake_sent_at(i).id) == HAND_AXIS) hand_frame = true;
  CHECK_FALSE(hand_frame);
}

TEST_CASE("a TRANSMITTING stage does count the lead clamp and the dev_over ticks") {
  reset_interp_test();
  seed_hand_encoder();
  axes[HAND_AXIS].heartbeat_seen = true;
  axes[0].heartbeat_seen = true; axes[0].pos_rev = 0.5f;
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());

  interp_set_output_enabled(true);
  uint16_t seq = 1;
  for (int k = 0; k < 5; ++k) {
    fake_advance(INTERP_PERIOD_US);
    write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
    stage_hand_far_from_encoder(seq++);
    interp_isr();
  }
  CHECK(interp_hand_lead_clamp_ticks() == 5u);
  CHECK(interp_hand_dev_over_ticks() == 5u);
  CHECK(interp_hand_sent() == 5u);
}

TEST_CASE("coldstart (homing_active) suppresses the hand counter gate exactly like output-disabled") {
  // The counter gate is `s_output_enabled && !coldstart`
  // (leg_interp.cpp's `out_en && !coldstart`). The output-disabled leg of the
  // gate is covered above (the ABORTED-stage test); this pins the OTHER leg —
  // homing_active() true with output ENABLED must also leave the counters at
  // zero, because a cold-start move (leg_homing's SETUP owning axis 6) holds
  // the streamed lane's command off the wire regardless of s_output_enabled.
  reset_interp_test();
  seed_hand_encoder();
  axes[HAND_AXIS].heartbeat_seen = true;
  axes[0].heartbeat_seen = true; axes[0].pos_rev = 0.5f;
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());

  interp_set_output_enabled(true);
  fake_set_homing(true);
  uint16_t seq = 1;
  for (int k = 0; k < 20; ++k) {
    fake_advance(INTERP_PERIOD_US);
    write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
    stage_hand_far_from_encoder(seq++);
    interp_isr();
  }
  // Non-vacuity: the lane really did compute a huge residual throughout —
  // it is only the CUMULATIVE counters that coldstart gates.
  CHECK(interp_hand_dev_last() > MAX_DEVIATION_HAND_REV);
  CHECK(interp_hand_lead_clamp_ticks() == 0u);
  CHECK(interp_hand_dev_over_ticks() == 0u);
  CHECK(interp_hand_sent() == 0u);

  // Coldstart falling is the same kind of edge as output re-enabling: the very
  // next tick counts.
  fake_set_homing(false);
  fake_advance(INTERP_PERIOD_US);
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
  stage_hand_far_from_encoder(seq++);
  interp_isr();
  CHECK(interp_hand_lead_clamp_ticks() == 1u);
  CHECK(interp_hand_dev_over_ticks() == 1u);
  CHECK(interp_hand_sent() == 1u);
}

TEST_CASE("hand7 reset zeroes the counters and residual, and NOT the arm state") {
  reset_interp_test();
  seed_hand_encoder();
  axes[HAND_AXIS].heartbeat_seen = true;
  axes[0].heartbeat_seen = true; axes[0].pos_rev = 0.5f;
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
  interp_set_output_enabled(true);
  uint16_t seq = 1;
  for (int k = 0; k < 4; ++k) {
    fake_advance(INTERP_PERIOD_US);
    write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
    stage_hand_far_from_encoder(seq++);
    interp_isr();
  }
  interp_set_hand_dev_guard_armed(true);              // the second sitting's state
  REQUIRE(interp_hand_lead_clamp_ticks() > 0u);
  REQUIRE(interp_hand_dev_over_ticks() > 0u);
  REQUIRE(interp_hand_sent() > 0u);

  CHECK(interp_hand7_console("hand7 reset"));         // the verb is recognised

  CHECK(interp_hand_lead_clamp_ticks() == 0u);
  CHECK(interp_hand_dev_over_ticks() == 0u);
  CHECK(interp_hand_sent() == 0u);
  CHECK(interp_hand_dev_last() == doctest::Approx(0.0f));
  CHECK(interp_hand_dev_max() == doctest::Approx(0.0f));
  CHECK(interp_hand_dev_trip_dev() == doctest::Approx(0.0f));
  // A diagnostic verb must never be a control action: the guard stays ARMED and
  // the lane keeps its knot state, so a reset mid-sitting cannot silently
  // disarm the E-STOP the sitting is relying on.
  CHECK(interp_hand_dev_guard_armed());
  CHECK(interp_hand_lane_active());

  // And it keeps counting afterwards, from zero.
  fake_advance(INTERP_PERIOD_US);
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
  stage_hand_far_from_encoder(seq++);
  interp_isr();
  CHECK(interp_hand_lead_clamp_ticks() == 1u);
}

TEST_CASE("hand7 reset does not swallow an unrelated console line") {
  CHECK_FALSE(interp_hand7_console("hand7 resetx"));
  CHECK_FALSE(interp_hand7_console("gpio"));
}

TEST_CASE("hand7 observe lasts ONE armed session: the disarm edge re-arms the deviation guard (FW 21)") {
  // Owner decision 2026-09-11: the guard boots ARMED; `hand7 observe` is a bench
  // read, and the guard must never be left off for the NEXT session by an
  // operator who forgot `hand7 arm`. The re-arm is the true→false output-enable
  // edge, not the arm edge, so the observed session itself stays observed.
  interp_reset();
  CHECK(interp_hand_dev_guard_armed() == true);            // power-on value
  interp_set_hand_dev_guard_armed(false);                  // `hand7 observe` before the stage
  interp_set_output_enabled(true);                         // arm edge: still observing
  CHECK(interp_hand_dev_guard_armed() == false);
  interp_set_output_enabled(true);                         // idempotent re-assert: no edge
  CHECK(interp_hand_dev_guard_armed() == false);
  interp_set_output_enabled(false);                        // disarm edge: back to ARMED
  CHECK(interp_hand_dev_guard_armed() == true);
  interp_set_output_enabled(false);                        // no edge, stays ARMED
  CHECK(interp_hand_dev_guard_armed() == true);
  interp_set_hand_dev_guard_armed(false);                  // `hand7 observe` while disarmed...
  interp_set_output_enabled(true);
  CHECK(interp_hand_dev_guard_armed() == false);           // ...covers the next session...
  interp_set_output_enabled(false);
  CHECK(interp_hand_dev_guard_armed() == true);            // ...and only that one.
}

// ═══ FW 22 scheduled (stamped) playback ══════════════════════════════════════
// The compiled mirror of tests/firmware/test_sched_c2_twin.py's key scenarios.
// Float32 on the host, so these assert against the analytic Hermite with a
// float tolerance, not bit-exactness (the twin carries the float64 claims).

// A C2 knot plan: accel linear between knots (the twin's c2_plan).
struct C2Plan { double p[64], v[64], a[64]; };
static C2Plan make_c2_plan(double amp_a, int period, double p0) {
  C2Plan P{};
  const double T = SEGMENT_T_S;
  for (int k = 0; k < 64; ++k) P.a[k] = amp_a * std::sin(2.0 * M_PI * k / period);
  P.v[0] = -amp_a * period * T / (2.0 * M_PI);
  P.p[0] = p0;
  for (int k = 0; k < 63; ++k) {
    P.v[k + 1] = P.v[k] + T * (P.a[k] + P.a[k + 1]) / 2.0;
    P.p[k + 1] = P.p[k] + T * P.v[k] + T * T * (P.a[k] / 3.0 + P.a[k + 1] / 6.0);
  }
  return P;
}
static void plan_ref(const C2Plan& P, uint64_t t0, uint64_t now, double& p, double& v, double& a) {
  const double tau = (double)(int64_t)(now - t0) / (SEGMENT_T_S * 1e6);
  const int k = (int)std::floor(tau);
  const double s = tau - k, T = SEGMENT_T_S, d = P.p[k + 1] - P.p[k];
  const double s2 = s * s, s3 = s2 * s;
  p = P.p[k] + (s3 - 2 * s2 + s) * T * P.v[k] + (-2 * s3 + 3 * s2) * d + (s3 - s2) * T * P.v[k + 1];
  v = (-6 * s2 + 6 * s) * d / T + (3 * s2 - 4 * s + 1) * P.v[k] + (3 * s2 - 2 * s) * P.v[k + 1];
  a = ((6 - 12 * s) * d / T + (6 * s - 4) * P.v[k] + (6 * s - 2) * P.v[k + 1]) / T;
}

// Stage a stamped frame: hand knots k..k+2 of P, legs flat at 0.5 (clamp-free).
static void stage_sched_frame(uint16_t seq, const C2Plan& P, int k, uint64_t t0,
                              uint32_t flags = 0x3Fu /* U1|U2|HAND|V1|V2|SCHED */) {
  JbUdp::SetpointPayload sp;
  memset(&sp, 0, sizeof(sp));
  for (int i = 0; i < 6; ++i) { sp.u0[i] = sp.u1[i] = sp.u2[i] = 0.5f; }
  sp.u0[6] = (float)P.p[k];     sp.u1[6] = (float)P.p[k + 1]; sp.u2[6] = (float)P.p[k + 2];
  sp.v0[6] = (float)P.v[k];     sp.v1[6] = (float)P.v[k + 1]; sp.v2[6] = (float)P.v[k + 2];
  sp.accel[6] = (float)P.a[k];
  sp.flags = flags;
  sp.t_origin_us = t0 + (uint64_t)k * 25'000u;
  interp_on_setpoint(seq, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
}

// Drive ticks [from, to): ingest frames whose arrival <= now, ISR, then park the
// hand encoder on the command (keeps the lead clamp out of the comparison).
struct SchedRun {
  std::vector<std::pair<uint64_t, int>> ev;   // (arrival wall us, k)
  size_t ei = 0;
  uint16_t seq = 1;
};
static void sched_ticks(SchedRun& r, const C2Plan& P, uint64_t t0, uint64_t to,
                        const std::function<void(uint64_t)>& per_tick) {
  while (fake_wall_us() < to) {
    while (r.ei < r.ev.size() && r.ev[r.ei].first <= fake_wall_us()) stage_sched_frame(r.seq++, P, r.ev[r.ei++].second, t0);
    interp_isr();
    if (interp_hand_lane_active()) write_pos_vel(hand_axis(), axes[HAND_AXIS].target_pos_rev, 0.0f, fake_mono_us());
    per_tick(fake_wall_us());
    fake_advance(INTERP_PERIOD_US);
  }
}

TEST_CASE("FW 22 scheduled: stamped frames with late arrivals + a dropped frame replay the plan (hand pos, vel, a_cmd)") {
  reset_interp_test();
  seed_hand_encoder();
  const C2Plan P = make_c2_plan(1500.0, 12, 5.0);
  const uint64_t t0 = 1'100'000;   // wall stamp of knot 0 (fake wall == mono)
  write_pos_vel(hand_axis(), (float)P.p[0], 0.0f, fake_mono_us());   // encoder on the plan (lead clamp out of the way)
  SchedRun r;
  uint64_t prev = 0;
  for (int k = 0; k < 40; ++k) {
    if (k == 17) continue;                                        // one dropped frame
    const uint64_t late = (uint64_t)(1 + (k * 7919) % 15) * 1000u;   // 1..15 ms AFTER the stamp (emit lead 0)
    uint64_t arr = t0 + (uint64_t)k * 25'000u + late;
    if (arr < prev) arr = prev;
    r.ev.push_back({arr, k});
    prev = arr;
  }
  uint32_t checked = 0;
  float max_da_step = 0.0f, prev_a = 0.0f;
  bool have_prev = false;
  sched_ticks(r, P, t0, t0 + 38u * 25'000u, [&](uint64_t now) {
    if (!interp_hand_lane_active()) return;
    CHECK(interp_sched_phase(1) == 1);                            // never exhausted
    double pr, vr, ar;
    plan_ref(P, t0, now, pr, vr, ar);
    // Frame 17 is dropped: frame 16's span-2 cubic continues (the grace) from 18T
    // until frame 18 lands (≤ 15 ms + one tick) — bounded there, exact elsewhere.
    const uint64_t g0 = t0 + 18u * 25'000u;
    if (now >= g0 && now < g0 + 18'000u) {
      CHECK(std::fabs(axes[HAND_AXIS].target_pos_rev - pr) < SCHED_RESUME_TOL_POS_HAND_REV);
      CHECK(std::fabs(axes[HAND_AXIS].target_vel_rps - vr) < SCHED_RESUME_TOL_VEL_HAND_RPS);
      CHECK(std::fabs(interp_hand_a_cmd() - ar) < SCHED_RESUME_TOL_ACC_HAND_RPS2);
    } else {
      CHECK(std::fabs(axes[HAND_AXIS].target_pos_rev - pr) < 2e-3);
      CHECK(std::fabs(axes[HAND_AXIS].target_vel_rps - vr) < 2e-2);
      CHECK(std::fabs(interp_hand_a_cmd() - ar) < 1.0);
    }
    const float a = interp_hand_a_cmd();
    if (have_prev && std::fabs(a - prev_a) > max_da_step) max_da_step = std::fabs(a - prev_a);
    prev_a = a; have_prev = true;
    ++checked;
  });
  CHECK(checked > 400);
  CHECK(interp_sched_phase(1) == 1);
  CHECK(interp_sched_stops() == 0);
  CHECK(interp_sched_expired() == 0);
  // plan jerk ≈ 1500·2π/12/0.025 ≈ 31 400 rev/s³ ⇒ ≤ ~63 rev/s² per 2 ms tick
  CHECK(max_da_step < 70.0f);
}

TEST_CASE("FW 22 scheduled: hand == leg parity on identical stamped knots (the parity anchor, scheduled path)") {
  reset_interp_test();
  seed_hand_encoder();
  write_pos_vel(hand_axis(), 0.52f, 0.0f, fake_mono_us());
  for (uint8_t i = 0; i < 6; ++i) { axes[i].pos_rev = 0.52f; axes[i].heartbeat_seen = true; }
  JbUdp::SetpointPayload sp;
  memset(&sp, 0, sizeof(sp));
  for (int i = 0; i < 7; ++i) {
    sp.u0[i] = 0.50f; sp.u1[i] = 0.55f; sp.u2[i] = 0.60f;
    sp.v0[i] = 2.0f;  sp.v1[i] = 2.0f;  sp.v2[i] = 2.0f;
  }
  sp.flags = 0x3Fu;
  sp.t_origin_us = fake_wall_us() + 3'000u;       // played 3 ms after arrival
  interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
  int play_ticks = 0;
  for (int k = 0; k < 30; ++k) {                  // spans both Hermite spans, then the stop
    interp_isr();
    // Parity is a PLAY-phase claim: once the cover ends each group stops under its
    // OWN limits (hand 3500 rev/s² vs legs 250), so the stop curves differ by design.
    if (interp_hand_lane_active() && interp_sched_phase(1) == 1) {
      ++play_ticks;
      CHECK(interp_sched_phase(0) == 1);
      CHECK(axes[HAND_AXIS].target_pos_rev == axes[0].target_pos_rev);
      CHECK(axes[HAND_AXIS].target_vel_rps == axes[0].target_vel_rps);
      CHECK(interp_hand_a_cmd() == interp_leg_a_cmd(0));
    }
    fake_advance(INTERP_PERIOD_US);
  }
  CHECK(play_ticks >= 24);
  CHECK(interp_hand_lane_active());
}

TEST_CASE("FW 22 scheduled: cover exhaustion runs a C2 stop, holds, and refuses a discontinuous resume while armed") {
  reset_interp_test();
  seed_hand_encoder();
  const C2Plan P = make_c2_plan(1500.0, 12, 5.0);
  const uint64_t t0 = 1'100'000;
  write_pos_vel(hand_axis(), (float)P.p[0], 0.0f, fake_mono_us());
  SchedRun r;
  const int K = 14;                                             // last frame; entry at knot 16
  for (int k = 0; k <= K; ++k) r.ev.push_back({t0 + (uint64_t)k * 25'000u - 20'000u, k});
  const uint64_t t_exh = t0 + (uint64_t)K * 25'000u + (uint64_t)SCHED_EXHAUST_US;   // cover + grace
  float prev_a = 0.0f, prev_v = 0.0f, max_da = 0.0f, peak_a_stop = 0.0f;
  bool have_prev = false;
  sched_ticks(r, P, t0, t_exh + 500'000u, [&](uint64_t now) {
    if (!interp_hand_lane_active()) return;
    const float a = interp_hand_a_cmd(), v = axes[HAND_AXIS].target_vel_rps;
    if (have_prev) {
      if (std::fabs(a - prev_a) > max_da) max_da = std::fabs(a - prev_a);
      CHECK(std::fabs(v - prev_v) <= 0.002f * std::max(std::fabs(a), std::fabs(prev_a)) + 0.5f);   // no velocity step
    }
    if (now >= t_exh && std::fabs(a) > peak_a_stop) peak_a_stop = std::fabs(a);
    prev_a = a; prev_v = v; have_prev = true;
  });
  CHECK(interp_sched_stops() == 2);                               // legs + hand groups
  CHECK(interp_sched_phase(1) == 3);
  CHECK(max_da <= SCHED_STOP_JERK_HAND_RPS3 * 0.002f * 1.01f + 1.0f);
  {  // entry accel = the grace-extended span (K+1 → K+2) at s = SCHED_S_MAX
    const double s = SCHED_S_MAX, T = SEGMENT_T_S, d = P.p[K + 2] - P.p[K + 1];
    const double a_entry = ((6 - 12 * s) * d / T + (6 * s - 4) * P.v[K + 1] + (6 * s - 2) * P.v[K + 2]) / T;
    CHECK(peak_a_stop <= std::max((double)SCHED_STOP_ACCEL_HAND_RPS2, std::fabs(a_entry)) * 1.001 + 1.0);
  }
  CHECK(axes[HAND_AXIS].target_vel_rps == 0.0f);
  CHECK(interp_hand_a_cmd() == 0.0f);
  const float hold = axes[HAND_AXIS].target_pos_rev;

  // Arm WITHOUT an edge (the edge would clear the lane): the refusal needs out_en.
  s_output_enabled = true;
  s_output_enabled_prev = true;
  const int k_res = 40;                                         // the plan resumes elsewhere
  const uint64_t tnow = fake_wall_us();
  const uint64_t t0_res = tnow + 5'000u - (uint64_t)k_res * 25'000u;
  stage_sched_frame(900, P, k_res, t0_res);
  for (int j = 0; j < 20; ++j) {
    interp_isr();
    CHECK(axes[HAND_AXIS].target_pos_rev == doctest::Approx(hold).epsilon(1e-6));
    write_pos_vel(hand_axis(), hold, 0.0f, fake_mono_us());
    fake_advance(INTERP_PERIOD_US);
  }
  CHECK(interp_sched_refused() >= 1);
  CHECK(interp_sched_hold_latched());
  CHECK(interp_base_pos(HAND_AXIS) == (float)P.p[k_res]);       // the guard reads the refused command

  // Disarmed: nothing reaches the wire, so the next frame is accepted and unlatches.
  s_output_enabled = false;
  s_output_enabled_prev = false;
  stage_sched_frame(901, P, k_res + 2, t0_res);
  for (int j = 0; j < 15; ++j) { interp_isr(); fake_advance(INTERP_PERIOD_US); }
  CHECK_FALSE(interp_sched_hold_latched());
  CHECK(interp_sched_phase(1) == 1);
}

TEST_CASE("FW 22 scheduled: HAS_SCHED without the full knot set, or on an unsynced clock, is demoted to legacy (counted)") {
  reset_interp_test();
  seed_hand_encoder();
  const C2Plan P = make_c2_plan(1500.0, 12, 5.0);
  stage_sched_frame(1, P, 0, fake_wall_us(), 0x2Fu);            // no HAS_V2
  interp_isr();
  CHECK(interp_sched_demoted() == 1);
  CHECK(interp_sched_phase(1) == 0);
  CHECK(interp_hand_lane_active());                              // latched through the legacy path
  CHECK(interp_hand_ff_gain_target() == 0.0f);                   // legacy targets K = 0
  fake_set_time_synced(false);
  stage_sched_frame(2, P, 1, fake_wall_us());
  interp_isr();
  CHECK(interp_sched_demoted() == 2);
  CHECK(interp_sched_phase(1) == 0);
  CHECK(interp_sched_frames() == 0);
}

TEST_CASE("FW 22 scheduled: a stamp beyond SCHED_MAX_FUTURE_US is refused and does not feed the staleness clock") {
  reset_interp_test();
  const C2Plan P = make_c2_plan(1500.0, 12, 5.0);
  const uint64_t before = interp_last_setpoint_us();
  stage_sched_frame(1, P, 0, fake_wall_us() + (uint64_t)SCHED_MAX_FUTURE_US + 10'000u);
  CHECK(interp_sched_frames() == 0);
  CHECK(interp_last_setpoint_us() == before);
}

// ═══════════════════════════════════════════════════════════════════════════
//  FW 22 hand acceleration torque feedforward (U2b)
//    tau = fade * sat(Ks * J * 2pi * a_cmd + bias), HAND_TOR_SCALE 1000, drain on TX stop
// ═══════════════════════════════════════════════════════════════════════════
static void stage_sched_frame_ff(uint16_t seq, const C2Plan& P, int k, uint64_t t0, float K, float bias) {
  JbUdp::SetpointPayload sp;
  memset(&sp, 0, sizeof(sp));
  for (int i = 0; i < 6; ++i) { sp.u0[i] = sp.u1[i] = sp.u2[i] = 0.5f; }
  sp.u0[6] = (float)P.p[k];     sp.u1[6] = (float)P.p[k + 1]; sp.u2[6] = (float)P.p[k + 2];
  sp.v0[6] = (float)P.v[k];     sp.v1[6] = (float)P.v[k + 1]; sp.v2[6] = (float)P.v[k + 2];
  sp.accel[6] = (float)P.a[k];
  sp.torque_ff[6] = bias;
  sp.hand_ff_gain = K;
  sp.flags = 0x3Fu;
  sp.t_origin_us = t0 + (uint64_t)k * 25'000u;
  interp_on_setpoint(seq, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
}

struct FfTick { uint64_t now; float tau, a; bool hand_frame; int16_t tq; uint8_t buf[8]; };
// Frames arrive 24 ms before their stamp (emit lead 1); output ENABLED; the hand
// encoder parks on the command each tick unless `pre` overrides it.
static std::vector<FfTick> ff_run(const C2Plan& P, uint64_t t0, int nframes, float K, float bias, uint64_t to,
                                  const std::function<void(uint64_t)>& pre = nullptr,
                                  const std::function<void(uint64_t)>& post = nullptr) {
  std::vector<FfTick> out;
  int k = 0; uint16_t seq = 1;
  while (fake_wall_us() < to) {
    while (k < nframes && t0 + (uint64_t)k * 25'000u <= fake_wall_us() + 24'000u) stage_sched_frame_ff(seq++, P, k++, t0, K, bias);
    fake_clear_sent();
    interp_isr();
    FfTick t{}; t.now = fake_wall_us(); t.tau = interp_hand_tau_cmd(); t.a = interp_hand_a_cmd();
    for (size_t i = 0; i < fake_sent_count(); ++i)
      if (ODrive::axis_of(fake_sent_at(i).id) == HAND_AXIS) {
        t.hand_frame = true; memcpy(t.buf, fake_sent_at(i).buf, 8); memcpy(&t.tq, &t.buf[6], 2);
      }
    out.push_back(t);
    if (interp_hand_lane_active()) write_pos_vel(hand_axis(), axes[HAND_AXIS].target_pos_rev, 0.0f, fake_mono_us());
    if (pre) pre(fake_wall_us());
    if (post) post(fake_wall_us());
    fake_advance(INTERP_PERIOD_US);
  }
  return out;
}
static void ff_setup(const C2Plan& P) {
  reset_interp_test();
  seed_hand_encoder();
  write_pos_vel(hand_axis(), (float)P.p[0], 0.0f, fake_mono_us());
  interp_set_output_enabled(true);
}
static float max_tau_step(const std::vector<FfTick>& v, size_t from = 1, size_t to = SIZE_MAX) {
  float m = 0.0f;
  for (size_t i = (from < 1 ? 1 : from); i < v.size() && i < to; ++i) m = std::max(m, std::fabs(v[i].tau - v[i - 1].tau));
  return m;
}

TEST_CASE("FW 22 hand torque FF: PLAY transmits tau = K*J*2pi*a_cmd at HAND_TOR_SCALE 1000, rate-bounded from 0") {
  const C2Plan P = make_c2_plan(1500.0, 12, 5.0);
  ff_setup(P);
  const uint64_t t0 = 1'100'000;
  const float K = 0.5f;
  const auto v = ff_run(P, t0, 44, K, 0.0f, t0 + 42u * 25'000u);
  CHECK(HAND_TOR_SCALE == 1000.0f);
  size_t checked = 0;
  for (const auto& t : v) {
    if (t.now < t0 + 300'000u) continue;              // Ks slew (100 ms) + fade (20 ms) long done
    REQUIRE(t.hand_frame);
    CHECK(std::fabs(t.tau - K * HAND_TORQUE_FF_J_2PI * t.a) < 1e-6f);
    CHECK(t.tq == (int16_t)lroundf(t.tau * HAND_TOR_SCALE));
    CHECK(axes[HAND_AXIS].target_torque_Nm == axes[HAND_AXIS].target_torque_Nm);   // finite
    ++checked;
  }
  CHECK(checked > 300);
  CHECK(interp_hand_ff_ks() == K);
  CHECK(interp_hand_ff_fade() == 1.0f);
  // Per-tick bound: K*J2pi*(plan jerk*dt ≈ 63 rev/s^2) + J2pi*1500*kstep ≈ 2.1e-3 + 1.0e-3 N m.
  CHECK(max_tau_step(v) < 4e-3f);
  CHECK(interp_hand_tau_clamp_ticks() == 0u);
  CHECK_FALSE(interp_hand_torque_clamp_take());
}

TEST_CASE("FW 22 hand torque FF: a LEGACY stream transmits tau == 0 exactly (bias + gain ignored) and never drains") {
  reset_interp_test();
  seed_hand_encoder();
  write_pos_vel(hand_axis(), 3.0f, 0.0f, fake_mono_us());
  interp_set_output_enabled(true);
  size_t hand_frames = 0;
  for (int n = 0; n < 60; ++n) {
    JbUdp::SetpointPayload sp;
    memset(&sp, 0, sizeof(sp));
    for (int i = 0; i < 6; ++i) sp.u0[i] = sp.u1[i] = 0.5f;
    sp.u0[6] = 3.0f + 0.2f * n; sp.u1[6] = 3.2f + 0.2f * n; sp.v0[6] = 8.0f; sp.v1[6] = 8.0f;
    sp.accel[6] = 800.0f; sp.torque_ff[6] = 0.04f; sp.hand_ff_gain = 1.0f;
    sp.flags = 0x1u | 0x4u | 0x8u;                    // U1 | HAS_HAND | V1 — no HAS_SCHED
    interp_on_setpoint((uint16_t)(n + 1), reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    for (int k = 0; k < 12; ++k) {
      fake_clear_sent();
      interp_isr();
      CHECK(interp_hand_tau_cmd() == 0.0f);
      CHECK(axes[HAND_AXIS].target_torque_Nm == 0.0f);
      for (size_t i = 0; i < fake_sent_count(); ++i)
        if (ODrive::axis_of(fake_sent_at(i).id) == HAND_AXIS) {
          ++hand_frames;
          CHECK(fake_sent_at(i).buf[6] == 0); CHECK(fake_sent_at(i).buf[7] == 0);
        }
      write_pos_vel(hand_axis(), axes[HAND_AXIS].target_pos_rev, 0.0f, fake_mono_us());
      fake_advance(INTERP_PERIOD_US);
    }
  }
  CHECK(hand_frames > 500);
  interp_set_output_enabled(false);
  for (int k = 0; k < 10; ++k) { fake_clear_sent(); interp_isr(); CHECK(fake_sent_count() == 0); fake_advance(INTERP_PERIOD_US); }
  CHECK(interp_hand_drain_frames() == 0u);
}

TEST_CASE("FW 22 hand torque FF: the sum saturates at ±HAND_TORQUE_FF_CLAMP_NM and latches heartbeat bit 14 (read-and-clear)") {
  REQUIRE(HAND_MOTOR_MAX_POSITION > 8.6f);
  const C2Plan P = make_c2_plan(3000.0, 8, 5.4);
  ff_setup(P);
  const uint64_t t0 = 1'100'000;
  const auto v = ff_run(P, t0, 40, 1.5f, 0.0f, t0 + 38u * 25'000u);
  float peak = 0.0f;
  for (const auto& t : v) peak = std::max(peak, std::fabs(t.tau));
  CHECK(peak <= HAND_TORQUE_FF_CLAMP_NM + 1e-6f);
  CHECK(peak >= HAND_TORQUE_FF_CLAMP_NM - 1e-6f);
  CHECK(interp_hand_tau_clamp_ticks() > 0u);
  CHECK(interp_hand_torque_clamp_take());
  CHECK_FALSE(interp_hand_torque_clamp_take());
  CHECK((int16_t)lroundf(HAND_TORQUE_FF_CLAMP_NM * HAND_TOR_SCALE) == 234);
}

TEST_CASE("FW 22 hand torque FF: DRAIN — output disable re-sends the last frame with torque 0 (3 ticks), pos/vel bit-identical") {
  const C2Plan P = make_c2_plan(1500.0, 12, 5.0);
  ff_setup(P);
  const uint64_t t0 = 1'100'000;
  uint64_t off_at = 0;
  const auto v = ff_run(P, t0, 44, 1.0f, 0.0f, t0 + 30u * 25'000u, nullptr, [&](uint64_t now) {
    if (off_at == 0 && now > t0 + 400'000u && std::fabs(interp_hand_tau_cmd()) > 0.02f) {
      off_at = now; interp_set_output_enabled(false);
    }
  });
  REQUIRE(off_at != 0);
  size_t i_off = 0;
  while (v[i_off].now != off_at) ++i_off;
  REQUIRE(v[i_off].hand_frame);
  CHECK(v[i_off].tq != 0);
  for (int d = 1; d <= 3; ++d) {
    REQUIRE(v[i_off + d].hand_frame);
    CHECK(v[i_off + d].tq == 0);
    CHECK(memcmp(v[i_off + d].buf, v[i_off].buf, 6) == 0);   // pos f32 + vel i16 unchanged
    CHECK(v[i_off + d].tau == 0.0f);
  }
  for (size_t i = i_off + 4; i < v.size(); ++i) CHECK_FALSE(v[i].hand_frame);
  CHECK(interp_hand_drain_frames() == 3u);
  CHECK(interp_hand_ff_ks() == 0.0f);
  CHECK(interp_hand_ff_fade() == 0.0f);
}

TEST_CASE("FW 22 hand torque FF: lead-clamp engage FADES tau to 0 (no step) and clear ramps it back") {
  const C2Plan P = make_c2_plan(1500.0, 12, 5.0);
  ff_setup(P);
  const uint64_t t0 = 1'100'000;
  const uint64_t e0 = t0 + 400'000u, e1 = t0 + 500'000u;
  const auto v = ff_run(P, t0, 44, 1.0f, 0.0f, t0 + 42u * 25'000u, [&](uint64_t now) {
    if (now >= e0 && now < e1) {
      double pr, vr, ar; plan_ref(P, t0, now + INTERP_PERIOD_US, pr, vr, ar);
      write_pos_vel(hand_axis(), (float)(pr - 3.0), 0.0f, fake_mono_us());   // 3 rev behind: lead clamp
    }
  });
  // fade 50 /s ⇒ 0.1·|raw| (≤ 0.1·J2pi·1500 ≈ 9.9e-3 N m) + K·J2pi·62 (≈ 4.1e-3) per tick
  CHECK(max_tau_step(v) < HAND_TORQUE_FF_J_2PI * (62.1f + 0.1f * 1500.0f) + 1e-4f);
  bool zero_seen = false;
  for (const auto& t : v) {
    if (t.now >= e0 + 30'000u && t.now < e1) { CHECK(t.tau == 0.0f); zero_seen = true; }
    if (t.now >= e1 + 40'000u && t.now < t0 + 40u * 25'000u) CHECK(std::fabs(t.tau - HAND_TORQUE_FF_J_2PI * t.a) < 1e-6f);
  }
  CHECK(zero_seen);
}

TEST_CASE("FW 22 hand torque FF: cover exhaustion → C2 stop → hold keeps tau continuous and ends at 0") {
  const C2Plan P = make_c2_plan(1500.0, 12, 5.0);
  ff_setup(P);
  const uint64_t t0 = 1'100'000;
  const float K = 1.0f;
  const uint32_t stops0 = interp_sched_stops();       // cumulative, global across BOTH groups
  const auto v = ff_run(P, t0, 20, K, 0.0f, t0 + 20u * 25'000u + 900'000u);
  CHECK(interp_sched_stops() - stops0 == 2u);         // hand group + the flat leg group both exhaust
  CHECK(interp_sched_phase(1) == 3);                  // HOLD
  const float bound = K * HAND_TORQUE_FF_J_2PI * (SCHED_STOP_JERK_HAND_RPS3 * (INTERP_PERIOD_US * 1e-6f)) + 5e-3f;
  CHECK(max_tau_step(v) < bound);
  CHECK(v.back().tau == 0.0f);
  CHECK(v.back().hand_frame);
}
