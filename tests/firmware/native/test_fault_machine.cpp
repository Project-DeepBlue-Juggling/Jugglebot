// =============================================================================
//  test_fault_machine.cpp — compiled test of the REAL fault state machine
// =============================================================================
//  Drives the actual compiled fault_machine.cpp (NOT a Python transcription) and
//  asserts the safety-critical behaviours hold as machine code on the build host:
//
//    * the soft-reset bounce-loop limiter (one auto-clear, then fatal),
//    * undervoltage gating + the uniform-UV-benign distinction,
//    * the deferred-stow 5 invariants (logbook 2026-05-19) — including the
//      terminal-IDLE completion handling, asserted via the recording
//      can_jugglebot_send (never command a dead bus; IDLE-all on stow complete),
//    * the present-axis freshness scoping (the single-leg-rig reconnect
//      dead-lock fix), and
//    * the recoverable motor-feedback-staleness output suppression.
//
//  It #includes BOTH fault_machine.cpp and leg_interp.cpp (they share no symbol
//  names, so one TU is ODR-clean) so it can drive the static interp_isr() to
//  complete a stow and observe the fault machine's terminal-IDLE handling.
//
//  It ALSO emits a firmware-anchored golden vector (`--emit-golden <path>`): the
//  subset of scenarios that controller/teensy_link/fault_logic.py models
//  (FaultEvaluator / DeferredStowLatch). tests/firmware/test_fault_logic.py
//  replays those goldens through fault_logic.py so the Jetson host mirror can no
//  longer drift from the firmware either (and tests/firmware/test_native_firmware.py
//  asserts a fresh emission still equals the committed golden, so a firmware
//  behaviour change must regenerate it deliberately).
//
//  SCOPE: validates DECISION LOGIC, not FreeRTOS/ISR concurrency or 500 Hz
//  timing — those remain on-hardware-replay gaps. See README.md.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT   // custom main (handles --emit-golden)
#include "doctest.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "axis_state.h"
#include "odrive_protocol.h"
#include "canbridge_config.h"
#include "udp_protocol.h"
#include "fake_hal.h"

// The two safety-critical TUs under test (ODR-clean: disjoint symbol names).
#include "leg_interp.cpp"
#include "fault_machine.cpp"

using namespace CanBridge;

static constexpr uint32_t ERR_UV       = 512;                          // ERR_DC_BUS_UNDER_VOLTAGE
static constexpr uint8_t  CMD_CLEAR    = ODriveCmd::clear_errors;      // 0x18
static constexpr uint8_t  CMD_SETSTATE = ODriveCmd::set_requested_state; // 0x07
static constexpr uint8_t  ST_IDLE      = (uint8_t)ODriveState::IDLE;        // 1
static constexpr uint8_t  ST_CL        = (uint8_t)ODriveState::CLOSED_LOOP; // 8

// ── Setup helpers ─────────────────────────────────────────────────────────────
static void clear_legs() {
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    axes[i].active_errors = 0;
    axes[i].disarm_reason = 0;
    axes[i].axis_state = ST_IDLE;
    axes[i].procedure_result = 0;
    axes[i].heartbeat_seen = false;
    axes[i].heartbeat_stale = false;
    axes[i].last_heartbeat_us = 0;
    axes[i].pos_rev = 0.0f;
    axes[i].vel_rps = 0.0f;
    axes[i].pos_timestamp_us = 0;
    axes[i].target_pos_rev = 0.0f;
    axes[i].target_vel_rps = 0.0f;
    axes[i].target_torque_Nm = 0.0f;
  }
}

static void reset_all() {
  fake_reset();
  clear_legs();
  fault_machine_init();
  interp_reset();
  fault_set_mpc_active(false);
}

// clear_errors_can() broadcasts one clear per axis over NUM_AXES (hand
// parity), so divide the raw clear-frame count by NUM_AXES to recover the number of
// clear ROUNDS. (Byte-identical golden vs the earlier legs-only /NUM_LEGS: every scenario
// here does <= 2 rounds and 7N/6 floors to N for N<=5, but /NUM_AXES is the correct
// divisor now that a round is NUM_AXES frames.)
static size_t clear_broadcasts() { return fake_sent_count_cmd(CMD_CLEAR) / NUM_AXES; }

// ── Error-eval (FaultEvaluator-modelable) scenarios ───────────────────────────
using Vec6u = std::array<uint32_t, 6>;
using Vec6b = std::array<uint8_t, 6>;
static Vec6u uni(uint32_t v) { return {v, v, v, v, v, v}; }
static Vec6b uni8(uint8_t v) { return {v, v, v, v, v, v}; }

struct ErrStep {
  std::string note;
  bool   notify_clear;   // call fault_notify_clear_errors() instead of evaluate
  bool   preset_fatal;   // set s_fatal_error before the evaluate
  Vec6u  active;
  Vec6u  disarm;
  Vec6b  state;
  // expected (the human-readable contract; the golden emits the ACTUAL run)
  bool   exp_fatal;
  bool   exp_uv;
  int    exp_soft_reset;
  int    exp_clear_calls;   // cumulative clear broadcasts
  bool   fatal_can_error = false;  // preset s_fatal_can_error before the evaluate (dead-bus)
};
struct ErrScenario { std::string name; std::vector<ErrStep> steps; };

struct ErrOut { bool fatal; bool uv; int soft_reset; int clear_calls; };

static std::vector<ErrOut> run_error_scenario(const ErrScenario& sc) {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  std::vector<ErrOut> out;
  for (const auto& st : sc.steps) {
    // Dead-bus preset. MUST be per-step: fault_step()'s watchdog clears
    // s_fatal_can_error each tick (all_present_legs_fresh is vacuously true with
    // zero present legs), and evaluate_errors runs FIRST inside fault_step so it
    // sees this preset when deciding whether clear_errors_can() may fire.
    s_fatal_can_error = st.fatal_can_error;
    if (st.notify_clear) {
      fault_notify_clear_errors();
    } else {
      for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        axes[i].active_errors = st.active[i];
        axes[i].disarm_reason = st.disarm[i];
        axes[i].axis_state = st.state[i];
      }
      if (st.preset_fatal) s_fatal_error = true;
      fault_step();
    }
    out.push_back({s_fatal_error, s_undervoltage_error,
                   (int)s_soft_reset_attempts, (int)clear_broadcasts()});
  }
  return out;
}

static std::vector<ErrScenario> error_scenarios() {
  std::vector<ErrScenario> v;
  // active error → fatal
  {
    Vec6u a = uni(0); a[2] = 0x40;
    v.push_back({"active_error_fatal", {
      {"active leg2", false, false, a, uni(0), uni8(ST_IDLE), true, false, 0, 0}}});
  }
  // disarm while CLOSED_LOOP → fatal
  {
    Vec6u d = uni(0); d[3] = 0x40;
    Vec6b s = uni8(ST_IDLE); s[3] = ST_CL;
    v.push_back({"disarm_closed_loop_fatal", {
      {"disarm leg3 @CL", false, false, uni(0), d, s, true, false, 0, 0}}});
  }
  // soft-reset one-shot then fatal, then operator clear refills the budget
  v.push_back({"soft_reset_one_shot", {
    {"disarm all @IDLE",    false, false, uni(0), uni(0x40), uni8(ST_IDLE), false, false, 1, 1},
    {"disarm persists",     false, false, uni(0), uni(0x40), uni8(ST_IDLE), true,  false, 1, 1},
    {"operator clear",      true,  false, uni(0), uni(0),     uni8(ST_IDLE), false, false, 0, 1},
    {"disarm again",        false, false, uni(0), uni(0x40), uni8(ST_IDLE), false, false, 1, 2}}});
  // undervoltage-only disarm recovers without fatal. The soft-reset path fires
  // first (budget→1, one broadcast) then the UV-recovery path's second
  // clear_errors_can() resets the budget back to 0 (two broadcasts total) — the
  // firmware AND fault_logic.py agree on this (both port can_node), so the
  // budget ends at 0, not 1. (A hand-expected 1 here is the classic mental-model
  // error this compiled harness exists to catch.)
  v.push_back({"uv_only_recovers", {
    {"uv disarm all @IDLE", false, false, uni(0), uni(ERR_UV), uni8(ST_IDLE), false, false, 0, 2}}});
  // active undervoltage error sets the flag AND is fatal
  {
    Vec6u a = uni(0); a[0] = ERR_UV;
    v.push_back({"active_uv_sets_flag", {
      {"active uv leg0", false, false, a, uni(0), uni8(ST_IDLE), true, true, 0, 0}}});
  }
  // a preset fatal clears when all axes are clean
  v.push_back({"all_clean_clears_fatal", {
    {"preset fatal, clean", false, true, uni(0), uni(0), uni8(ST_IDLE), false, false, 0, 0}}});
  // Dead-bus no-op clear PRESERVES the soft-reset budget. A disarm while the
  // CAN3 bus is confirmed down: clear_errors_can() is a no-op (never command a dead
  // bus), so the soft-reset budget is NOT consumed. When the bus returns the one
  // auto-clear finally fires (budget→1, one broadcast); the very next disarm then hits
  // the one-shot cap and goes fatal. Proves the budget survived the dead-bus tick
  // instead of being silently spent on a clear that never went out.
  v.push_back({"dead_bus_no_op_clear_preserves_budget", {
    {"dead bus disarm",   false, false, uni(0), uni(0x40), uni8(ST_IDLE), false, false, 0, 0, true},
    {"bus back, 1 clear", false, false, uni(0), uni(0x40), uni8(ST_IDLE), false, false, 1, 1, false},
    {"budget exhausted",  false, false, uni(0), uni(0x40), uni8(ST_IDLE), true,  false, 1, 1, false}}});
  return v;
}

// ── Deferred-stow (DeferredStowLatch-modelable) scenarios ─────────────────────
// The reboot-suppression latch adds two input columns: reboot_start (arm the latch,
// = the REBOOT_ODRIVES RPC) and deadline_pass (the reboot-suppression deadline has
// elapsed). The abstract (stale, fresh, deadline_pass) triple stands in for the
// firmware's clock-driven any_leg_heartbeat_stale / all_present_legs_fresh /
// now>=deadline, so the Jetson host mirror (fault_logic.py DeferredStowLatch) can
// replay these goldens clock-free.
struct StowStep {
  std::string note;
  bool stale;
  bool fresh;
  bool complete;       // drive interp_isr to set stow_complete before this fault_step
  bool reboot_start;   // reboot-latch input: fault_notify_reboot_started() before this fault_step
  bool deadline_pass;  // reboot-latch input: advance the clock past the reboot-suppression deadline
  bool exp_fatal;
  bool exp_pending;
  bool exp_stowing;
  bool seen = true;    // cold-start seeds a present leg heartbeat this step
};
struct StowScenario { std::string name; std::vector<StowStep> steps; bool cold_start = false; };
struct StowOut { bool fatal; bool pending; bool stowing; };

// In-window staleness advance: > CAN_HEARTBEAT_TIMEOUT_US (legs go stale) but
// < REBOOT_WATCHDOG_SUPPRESS_US (still inside the reboot-suppression window), so a
// "stale" step alone never crosses the reboot deadline — deadline_pass does that.
static constexpr uint64_t STALE_ADVANCE_US = CAN_HEARTBEAT_TIMEOUT_US + 500'000;  // 2.5 s

static std::vector<StowOut> run_stow_scenario(const StowScenario& sc) {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  if (!sc.cold_start) {
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {     // full robot: all 6 present
      axes[i].heartbeat_seen = true;
      axes[i].last_heartbeat_us = fake_mono_us();
    }
  }
  std::vector<StowOut> out;
  for (const auto& st : sc.steps) {
    // Cold-start first-seen gate: a leg is "present" only once its
    // heartbeat has been seen. seen=true seeds all legs fresh at this step's start
    // (so a following stale-advance can make them go stale); seen=false leaves the
    // bus never-seen, so s_first_leg_hb_seen gates detection OFF.
    if (sc.cold_start && st.seen) {
      for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        axes[i].heartbeat_seen = true;
        axes[i].last_heartbeat_us = fake_mono_us();
      }
    }
    if (st.reboot_start) fault_notify_reboot_started();               // arm the latch
    if (st.stale) fake_advance(STALE_ADVANCE_US);                     // legs go stale (in-window)
    if (st.deadline_pass) fake_advance(REBOOT_WATCHDOG_SUPPRESS_US);  // cross the reboot deadline
    if (st.fresh) for (uint8_t i = 0; i < NUM_LEGS; ++i) axes[i].last_heartbeat_us = fake_mono_us();
    if (st.complete) interp_isr();   // legs already at STOW (pos_rev=0) → one tick sets stow_complete
    fault_step();
    out.push_back({s_fatal_can_error, s_stow_pending, s_stowing});
  }
  return out;
}

static std::vector<StowScenario> stow_scenarios() {
  // cols: note, stale, fresh, complete, reboot_start, deadline_pass, exp_fatal, exp_pending, exp_stowing
  return {
    {"loss_arms_latch", {
      {"can3 loss",   true,  false, false, false, false, true,  true,  false}}},
    {"execute_on_reconnect", {
      {"can3 loss",   true,  false, false, false, false, true,  true,  false},
      {"reconnect",   false, true,  false, false, false, false, true,  true}}},
    {"rearm_on_redrop", {
      {"can3 loss",   true,  false, false, false, false, true,  true,  false},
      {"reconnect",   false, true,  false, false, false, false, true,  true},
      {"redrop",      true,  false, false, false, false, true,  true,  false}}},
    {"complete_clears_latch", {
      {"can3 loss",   true,  false, false, false, false, true,  true,  false},
      {"reconnect",   false, true,  false, false, false, false, true,  true},
      {"stow done",   false, true,  true,  false, false, false, false, false}}},
    // ── reboot-suppression latch─────────────────────────────────────
    {"reboot_suppresses_then_fresh_release", {
      // arm (legs fresh) → reboot silence SUPPRESSED (no false loss) → legs return →
      // latch releases → a NEW spontaneous loss then arms the stow normally.
      {"reboot armed",  false, false, false, true,  false, false, false, false},
      {"reboot silence",true,  false, false, false, false, false, false, false},
      {"legs back",     false, true,  false, false, false, false, false, false},
      {"new loss",      true,  false, false, false, false, true,  true,  false}}},
    {"reboot_deadline_release", {
      // arm → suppressed in-window → past the deadline the latch releases and the
      // still-stale loss (a reboot that never returned) is detected + stow armed.
      {"reboot armed",  false, false, false, true,  false, false, false, false},
      {"in-window",     true,  false, false, false, false, false, false, false},
      {"deadline",      true,  false, false, false, true,  true,  true,  false}}},
    {"reboot_latch_does_not_touch_spontaneous_loss", {
      // no reboot armed → a spontaneous loss detects immediately (inversion intact).
      {"spontaneous loss", true, false, false, false, false, true, true, false}}},
    // ── Gap 10: cold-start first-seen gate ───────────────────────────────────
    // The firmware AND-gates CAN-loss detection on s_first_leg_hb_seen (any leg
    // heartbeat EVER seen). On a cold boot — before any leg has reported — an
    // apparent staleness must NOT arm the stow; only AFTER the first heartbeat does
    // a subsequent loss detect. cold_start=true skips the pre-seed so the gate is
    // exercised from the true never-seen state (seen=false on boot).
    {"cold_start_first_seen_gate", {
      {"boot (never seen)", true,  false, false, false, false, false, false, false, false},
      {"first heartbeat",   false, true,  false, false, false, false, false, false, true},
      {"loss after seen",   true,  false, false, false, false, true,  true,  false, true}},
     true},
  };
}

// =============================================================================
//  doctest assertions — the compiled robustness contract
// =============================================================================
TEST_CASE("error-eval scenarios match the firmware contract") {
  for (const auto& sc : error_scenarios()) {
    CAPTURE(sc.name);
    auto out = run_error_scenario(sc);
    REQUIRE(out.size() == sc.steps.size());
    for (size_t k = 0; k < out.size(); ++k) {
      CAPTURE(sc.steps[k].note);
      CHECK(out[k].fatal       == sc.steps[k].exp_fatal);
      CHECK(out[k].uv          == sc.steps[k].exp_uv);
      CHECK(out[k].soft_reset  == sc.steps[k].exp_soft_reset);
      CHECK(out[k].clear_calls == sc.steps[k].exp_clear_calls);
    }
  }
}

TEST_CASE("deferred-stow latch scenarios match the firmware contract") {
  for (const auto& sc : stow_scenarios()) {
    CAPTURE(sc.name);
    auto out = run_stow_scenario(sc);
    REQUIRE(out.size() == sc.steps.size());
    for (size_t k = 0; k < out.size(); ++k) {
      CAPTURE(sc.steps[k].note);
      CHECK(out[k].fatal   == sc.steps[k].exp_fatal);
      CHECK(out[k].pending == sc.steps[k].exp_pending);
      CHECK(out[k].stowing == sc.steps[k].exp_stowing);
    }
  }
}

TEST_CASE("never command a dead bus + terminal IDLE on stow complete") {
  // Walk the full loss→reconnect→complete arc and watch the recording HAL.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    axes[i].heartbeat_seen = true;
    axes[i].last_heartbeat_us = fake_mono_us();
  }
  // Loss: detection arms the latch and gates output OFF — and the fault machine
  // emits NOTHING to the dead bus (no clears: legs are error-free).
  fake_advance(3 * (uint64_t)CAN_HEARTBEAT_TIMEOUT_US);
  fake_clear_sent();
  fault_step();
  CHECK(s_fatal_can_error);
  CHECK(s_stow_pending);
  CHECK(interp_output_enabled() == false);     // never command the dead bus (gate)
  CHECK(fake_sent_count() == 0);                // ...and nothing was sent to it
  CHECK(fault_state() == JbUdp::FaultState::CAN_BUS_DOWN);

  // Confirmed reconnect: the profiled stow begins, output gate re-enabled.
  for (uint8_t i = 0; i < NUM_LEGS; ++i) axes[i].last_heartbeat_us = fake_mono_us();
  fault_step();
  CHECK(s_fatal_can_error == false);
  CHECK(s_stowing);
  CHECK(interp_stow_active());
  CHECK(interp_output_enabled());

  // Drive the descent to completion (legs already at the off pose), then the
  // fault machine IDLEs every leg and clears the latch.
  interp_isr();                                 // one tick → stow_complete
  CHECK(interp_stow_complete());
  fake_clear_sent();
  fault_step();
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == NUM_LEGS);   // IDLE all six
  CHECK(s_stowing == false);
  CHECK(s_stow_pending == false);
}

TEST_CASE("present-axis freshness: single-leg rig reconnect is NOT dead-locked") {
  // Only leg 0 present (the odrv0-only bench rig). The earlier all-six
  // freshness form would dead-lock the reconnect (legs 1-5 never report fresh);
  // the present-scoped form fires on leg 0 alone.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  axes[0].heartbeat_seen = true;
  axes[0].last_heartbeat_us = fake_mono_us();
  // Loss: leg 0 goes stale → detection fires even with five absent legs.
  fake_advance(3 * (uint64_t)CAN_HEARTBEAT_TIMEOUT_US);
  fault_step();
  CHECK(s_fatal_can_error);
  CHECK(s_stow_pending);
  // Reconnect: leg 0 fresh again → stow EXECUTES (the dead-lock fix).
  axes[0].last_heartbeat_us = fake_mono_us();
  fault_step();
  CHECK(s_fatal_can_error == false);
  CHECK(s_stowing);
}

TEST_CASE("motor-feedback staleness suppresses output recoverably (not latched)") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fault_set_mpc_active(true);
  axes[0].heartbeat_seen = true;
  axes[0].last_heartbeat_us = fake_mono_us();
  axes[0].pos_rev = 0.0f;
  axes[0].pos_timestamp_us = fake_mono_us();
  fake_set_udp_last_rx_us(fake_mono_us());
  // Latch a (zero) setpoint so interp_have_latched() is true (the guard's pre-arm
  // gate); u0=0 so there is no max-deviation trip.
  JbUdp::SetpointPayload sp;
  memset(&sp, 0, sizeof(sp));
  interp_on_setpoint(0, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
  interp_isr();
  REQUIRE(interp_have_latched());

  // Feedback freezes: advance 0.2 s (> MOTOR_FB_STALENESS 0.15 s) without a new
  // encoder timestamp; keep the link + command fresh so nothing else trips.
  fake_advance(200'000);
  axes[0].last_heartbeat_us = fake_mono_us();   // bus still up
  fake_set_udp_last_rx_us(fake_mono_us());      // link still up
  fault_step();
  CHECK(fault_state() == JbUdp::FaultState::MOTOR_FB_STALE);
  CHECK(interp_output_enabled() == false);      // output suppressed

  // Feedback returns → output re-enables (recoverable, NOT a latched E-STOP).
  axes[0].pos_timestamp_us = fake_mono_us();
  fault_step();
  CHECK(fault_state() != JbUdp::FaultState::MOTOR_FB_STALE);
  CHECK(interp_output_enabled());
}

TEST_CASE("guard E-STOP / fb-stale are present-scoped and pre-arm-safe") {
  // MAX_DEVIATION: a PRESENT leg whose commanded base diverges > MAX_DEVIATION_REV
  // from its encoder trips the E-STOP; an ABSENT leg with a far command does NOT
  // (production sends 6 targets; an absent node reads pos_rev=0 and must be skipped).
  auto arm_leg0 = []() {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    fault_set_mpc_active(true);
    axes[0].heartbeat_seen = true;
    axes[0].last_heartbeat_us = fake_mono_us();
    axes[0].pos_rev = 0.0f;
    axes[0].pos_timestamp_us = fake_mono_us();
    fake_set_udp_last_rx_us(fake_mono_us());
  };
  auto latch_u0 = [](float a0, float a1) {
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
    sp.u0[0] = a0; sp.u0[1] = a1;     // leg1 is absent in these cases
    interp_on_setpoint(0, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
  };

  SUBCASE("present leg diverging trips MAX_DEVIATION") {
    arm_leg0();
    latch_u0(0.9f, 3.0f);             // leg0 (present) diverges 0.9 > 0.5; leg1 (absent) far
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::MAX_DEVIATION);
  }
  SUBCASE("absent leg's far command alone does NOT trip MAX_DEVIATION") {
    arm_leg0();
    latch_u0(0.0f, 3.0f);             // leg0 aligned; only the absent leg1 is far
    fault_step();
    CHECK(fault_state() != JbUdp::FaultState::MAX_DEVIATION);
  }
  SUBCASE("fb-stale does not false-trip pre-arm (mpc inactive)") {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    fault_set_mpc_active(false);      // not armed
    axes[0].heartbeat_seen = true;
    axes[0].last_heartbeat_us = fake_mono_us();
    axes[0].pos_timestamp_us = 0;     // ancient feedback timestamp
    fake_advance(1'000'000);
    axes[0].last_heartbeat_us = fake_mono_us();
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    CHECK(fault_state() != JbUdp::FaultState::MOTOR_FB_STALE);
  }
}

// =============================================================================
//  Guard E-STOP latch — motor_guard sticky-self.mode semantics
// =============================================================================
TEST_CASE("guard E-STOP LATCHES until an explicit operator clear") {
  // Arm the guard: mpc_active, leg0 present+fresh, link fresh, a latched setpoint
  // (so interp_have_latched() is true for the MAX_DEVIATION path).
  auto arm = []() {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    fault_set_mpc_active(true);
    axes[0].heartbeat_seen = true;
    axes[0].last_heartbeat_us = fake_mono_us();
    axes[0].pos_rev = 0.0f;
    axes[0].pos_timestamp_us = fake_mono_us();
    fake_set_udp_last_rx_us(fake_mono_us());
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
    interp_on_setpoint(0, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
    fake_set_udp_last_rx_us(fake_mono_us());   // keep the link fresh after the tick
  };

  SUBCASE("overspeed latches, PERSISTS after the condition clears, releases only on clear-errors") {
    arm();
    axes[0].vel_rps = MAX_MOTOR_VEL_RPS + 1.0f;   // overspeed
    fault_step();
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    CHECK(fault_state() == JbUdp::FaultState::MOTOR_OVERSPEED);
    CHECK(interp_output_enabled() == false);

    // THE bug-catcher: clear the condition — pre-fix the guard reverts to ENABLED
    // and output re-enables the very next tick. The latch must hold.
    axes[0].vel_rps = 0.0f;
    for (int k = 0; k < 5; ++k) { fake_set_udp_last_rx_us(fake_mono_us()); fault_step(); }
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    CHECK(fault_state() == JbUdp::FaultState::MOTOR_OVERSPEED);   // reason frozen at the trip
    CHECK(interp_output_enabled() == false);

    // Operator clear releases it → clean re-enable (condition truly gone).
    fault_notify_clear_errors();
    fault_step();
    CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);
    CHECK(interp_output_enabled() == true);
  }

  SUBCASE("cannot clear THROUGH a live condition (re-latches on the same step)") {
    arm();
    axes[0].vel_rps = MAX_MOTOR_VEL_RPS + 1.0f;
    fault_step();
    REQUIRE(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    // Clear while the condition is STILL live → re-trips + re-latches immediately.
    fault_notify_clear_errors();
    fault_step();
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    CHECK(interp_output_enabled() == false);
  }

  SUBCASE("MAX_DEVIATION latches + persists after the divergence shrinks") {
    arm();
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
    sp.u0[0] = 0.9f;                              // leg0 base diverges 0.9 > MAX_DEVIATION_REV
    interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    REQUIRE(fault_state() == JbUdp::FaultState::MAX_DEVIATION);
    REQUIRE(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    // Shrink the divergence (encoder catches up). Latch must hold.
    axes[0].pos_rev = 0.9f;
    for (int k = 0; k < 3; ++k) { fake_set_udp_last_rx_us(fake_mono_us()); fault_step(); }
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    fault_notify_clear_errors();
    fault_step();
    CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);
  }

  SUBCASE("an INTERNAL clear (soft-reset/UV auto-retry) does NOT release the latch") {
    arm();
    axes[0].vel_rps = MAX_MOTOR_VEL_RPS + 1.0f;
    fault_step();
    REQUIRE(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    axes[0].vel_rps = 0.0f;                       // condition gone
    // Drive a UV-only disarm on all legs @IDLE → evaluate_errors' soft-reset/UV path
    // calls clear_errors_can()→clear_error_flags() (NOT fault_notify_clear_errors).
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      axes[i].disarm_reason = ERR_UV;
      axes[i].axis_state = ST_IDLE;
    }
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    // The internal clear must NOT have released the guard E-STOP.
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    CHECK(interp_output_enabled() == false);
  }
}

// =============================================================================
//  Reboot-suppression latch— explicit readable safety contract
// =============================================================================
TEST_CASE("reboot latch: a SPONTANEOUS CAN loss is UNAFFECTED (deferred-stow inversion preserved)") {
  // THE safety-critical proof: a loss NOT preceded by a REBOOT_ODRIVES must detect and
  // arm the deferred stow exactly as before — the latch only touches reboot-armed silence.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) { axes[i].heartbeat_seen = true; axes[i].last_heartbeat_us = fake_mono_us(); }
  // NO fault_notify_reboot_started().
  fake_advance(3 * (uint64_t)CAN_HEARTBEAT_TIMEOUT_US);
  fault_step();
  CHECK(fault_can_bus_down());                              // detection fired
  CHECK(fault_stow_pending());                             // deferred stow armed (inversion intact)
  CHECK(fault_state() == JbUdp::FaultState::CAN_BUS_DOWN);
}

TEST_CASE("reboot latch: suppresses in-window, releases on fresh-after-stale AND on deadline") {
  // (a) fresh-after-stale release — the ODrives came back.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) { axes[i].heartbeat_seen = true; axes[i].last_heartbeat_us = fake_mono_us(); }
  fault_notify_reboot_started();                            // arm (legs fresh at arm time)
  fake_advance(CAN_HEARTBEAT_TIMEOUT_US + 500'000);         // 2.5 s: legs stale, < 6 s deadline
  fault_step();
  CHECK(fault_can_bus_down() == false);                    // SUPPRESSED — no false loss
  CHECK(fault_stow_pending() == false);                    // ...and no stow armed
  for (uint8_t i = 0; i < NUM_LEGS; ++i) axes[i].last_heartbeat_us = fake_mono_us();  // legs return
  fault_step();
  CHECK(fault_can_bus_down() == false);                    // released cleanly, no stow
  CHECK(fault_stow_pending() == false);

  // (b) deadline release — a reboot that never returns is still caught at the deadline.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) { axes[i].heartbeat_seen = true; axes[i].last_heartbeat_us = fake_mono_us(); }
  fault_notify_reboot_started();
  fake_advance(CAN_HEARTBEAT_TIMEOUT_US + 500'000);         // 2.5 s: in-window → suppressed
  fault_step();
  CHECK(fault_can_bus_down() == false);
  fake_advance(REBOOT_WATCHDOG_SUPPRESS_US);                // past the 6 s deadline, legs still silent
  fault_step();
  CHECK(fault_can_bus_down());                             // deadline backstop → real loss detected
  CHECK(fault_stow_pending());
}

TEST_CASE("reboot latch: a latch armed while legs are STILL fresh cannot release early") {
  // saw_stale gate: the reboot frames have not yet silenced the legs, so the same
  // freshness that was present at arm time must NOT release the latch — otherwise the
  // subsequent reboot silence would false-trip. Advance a little (still fresh), step,
  // then confirm the latch is still suppressing when the silence finally arrives.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) { axes[i].heartbeat_seen = true; axes[i].last_heartbeat_us = fake_mono_us(); }
  fault_notify_reboot_started();                            // arm (legs fresh)
  fault_step();                                             // legs still fresh → must NOT release
  fake_advance(CAN_HEARTBEAT_TIMEOUT_US + 500'000);         // now the reboot silence arrives (in-window)
  fault_step();
  CHECK(fault_can_bus_down() == false);                    // still suppressed (early release avoided)
}

// =============================================================================
//  Monotonic clock discipline
// =============================================================================
//  THE proof the fix works: set_wall_anchor() STEPS now_wall_us() (NTP-style,
//  forward OR backward, unbounded on re-acquisition after teensy_bridge_node was
//  down). Every staleness/guard/stow interval now reads micros64() (the pure
//  crystal a wall step never touches), so a wall STEP must be inert. The fake HAL
//  exposes independent wall/mono via fake_set_clock(wall_us, mono_us); production
//  stamps + reads all intervals on the mono clock, so stepping wall alone changes
//  nothing. The positive control proves staleness STILL fires on the mono clock.
TEST_CASE("wall step does not perturb staleness/guard/stow (wall/mono clock separation)") {
  const uint64_t W0 = 2'000'000'000ULL;   // wall base (>> the 5 s step, no underflow)
  const uint64_t M0 =    10'000'000ULL;   // mono base (independent of wall)

  // Arm the healthy, streaming baseline: mpc active, all legs present+fresh, link
  // fresh, a zero setpoint latched — everything stamped on the MONO clock (M0).
  auto arm_fresh = [&]() {
    reset_all();
    fake_set_clock(W0, M0);
    fault_set_mpc_active(true);
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      axes[i].heartbeat_seen    = true;
      axes[i].last_heartbeat_us = fake_mono_us();
      axes[i].pos_rev           = 0.0f;
      axes[i].pos_timestamp_us  = fake_mono_us();
    }
    fake_set_udp_last_rx_us(fake_mono_us());
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
    interp_on_setpoint(0, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));  // recv stamped micros64()
    interp_isr();
  };

  arm_fresh();
  fault_step();
  const uint8_t base_state   = fault_state();
  const uint8_t base_guard   = fault_guard_mode();
  const bool    base_output  = interp_output_enabled();
  const bool    base_pending = fault_stow_pending();
  // The armed baseline is the healthy ENABLED / streaming state.
  REQUIRE(base_state   == JbUdp::FaultState::NONE);
  REQUIRE(base_guard   == JbUdp::GuardMode::ENABLED);
  REQUIRE(base_output  == true);
  REQUIRE(base_pending == false);

  SUBCASE("5 s BACKWARD wall step (mono unchanged) is inert") {
    fake_set_clock(W0 - 5'000'000ULL, M0);   // wall jumps back 5 s; mono frozen at M0
    fault_step();
    CHECK(fault_state()           == base_state);
    CHECK(fault_guard_mode()      == base_guard);
    CHECK(interp_output_enabled() == base_output);
    CHECK(fault_stow_pending()    == base_pending);
  }
  SUBCASE("5 s FORWARD wall step (mono unchanged) is inert") {
    fake_set_clock(W0 + 5'000'000ULL, M0);   // wall jumps forward 5 s; mono frozen at M0
    fault_step();
    CHECK(fault_state()           == base_state);
    CHECK(fault_guard_mode()      == base_guard);
    CHECK(interp_output_enabled() == base_output);
    CHECK(fault_stow_pending()    == base_pending);
  }
  SUBCASE("positive control: a MONO advance past the timeout DOES trip CAN_BUS_DOWN") {
    fake_advance(3 * (uint64_t)CAN_HEARTBEAT_TIMEOUT_US);   // both clocks move → real staleness on mono
    fault_step();
    CHECK(fault_can_bus_down());
    CHECK(fault_state() == JbUdp::FaultState::CAN_BUS_DOWN);
  }
}

// =============================================================================
//  clear_disarm_reasons: a CLEAR zeroes the disarm cache
// =============================================================================
//  The firmware disarm_reason cache is written ONLY by the Get_Error RX decode, so a
//  CLEAR that did not also zero it would leave a stale nonzero disarm until the next
//  Get_Error frame — and evaluate_errors would read the stale disarm (any_disarmed) and
//  re-fault the state the operator just cleared. Both CLEAR paths (external RPC =
//  fault_notify_clear_errors, internal soft-reset auto-retry = clear_errors_can) must
//  mirror can_node._clear_errors → clear_disarm_reasons() over ALL axes incl. the hand.
TEST_CASE("clear_disarm_reasons: a CLEAR zeroes the disarm cache over all axes incl. the hand") {
  SUBCASE("external operator CLEAR (fault_notify_clear_errors) zeroes disarm immediately") {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    for (uint8_t i = 0; i < NUM_AXES; ++i) axes[i].disarm_reason = 0x40;   // legs 0-5 AND hand (6)
    fault_notify_clear_errors();
    for (uint8_t i = 0; i < NUM_AXES; ++i) { CAPTURE(i); CHECK(axes[i].disarm_reason == 0); }
  }
  SUBCASE("internal soft-reset auto-clear (clear_errors_can) zeroes disarm too") {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    // Disarm on every axis @IDLE, bus up → evaluate_errors' soft-reset branch fires
    // clear_errors_can() → clear_disarm_reasons(). Drive it via fault_step().
    for (uint8_t i = 0; i < NUM_AXES; ++i) { axes[i].disarm_reason = 0x40; axes[i].axis_state = ST_IDLE; }
    s_fatal_can_error = false;   // bus up so the clear actually sends
    fault_step();
    for (uint8_t i = 0; i < NUM_AXES; ++i) { CAPTURE(i); CHECK(axes[i].disarm_reason == 0); }
    // ...and the cache having been cleared, a follow-up step must NOT re-fault.
    fault_step();
    CHECK(s_fatal_error == false);
  }
}

// =============================================================================
//  hand-axis (axis 6) fault-eval scope: a hand fault ESTOPs the legs
// =============================================================================
//  evaluate_errors / clear now iterate NUM_AXES, so a hand active-error or hand
//  disarm-while-CLOSED_LOOP sets s_fatal_error / guard ESTOP (can_node parity — the
//  standing hand-gap-is-a-real-regression note). The CAN3 watchdog + deferred stow stay
//  legs-only, so a STALE hand heartbeat must NOT arm the leg watchdog/stow.
TEST_CASE("hand-axis fault eval: a hand fault ESTOPs (NUM_AXES scope)") {
  SUBCASE("hand active-error → ODRIVE_FATAL + guard ESTOP (legs clean)") {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    axes[HAND_AXIS].active_errors = 0x40;      // hand fault; all legs clean
    fault_step();
    CHECK(s_fatal_error);
    CHECK(fault_state() == JbUdp::FaultState::ODRIVE_FATAL);
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
  }
  SUBCASE("hand disarm while CLOSED_LOOP → fatal (legs clean)") {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    axes[HAND_AXIS].disarm_reason = 0x40;
    axes[HAND_AXIS].axis_state    = ST_CL;     // hand armed → disarm-while-CL is fatal
    fault_step();
    CHECK(s_fatal_error);
    CHECK(fault_state() == JbUdp::FaultState::ODRIVE_FATAL);
  }
  SUBCASE("NEGATIVE: a stale HAND heartbeat does NOT trip the CAN watchdog/stow (legs-only)") {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    // Only the hand has ever been seen; it then goes stale. Legs never seen.
    axes[HAND_AXIS].heartbeat_seen    = true;
    axes[HAND_AXIS].last_heartbeat_us = fake_mono_us();
    fake_advance(3 * (uint64_t)CAN_HEARTBEAT_TIMEOUT_US);   // hand heartbeat goes stale
    fault_step();
    CHECK(fault_can_bus_down() == false);   // watchdog/stow are legs-only (NUM_LEGS)
    CHECK(fault_stow_pending() == false);   // ...so a stale hand arms nothing
  }
}

// =============================================================================
//  Golden emission (firmware-anchored conformance source for fault_logic.py)
// =============================================================================
static void emit_vec6u(FILE* f, const Vec6u& v) {
  fprintf(f, "[%u,%u,%u,%u,%u,%u]", v[0], v[1], v[2], v[3], v[4], v[5]);
}
static void emit_vec6b(FILE* f, const Vec6b& v) {
  fprintf(f, "[%u,%u,%u,%u,%u,%u]", v[0], v[1], v[2], v[3], v[4], v[5]);
}
static const char* jb(bool b) { return b ? "true" : "false"; }

static int emit_golden(const char* path) {
  FILE* f = fopen(path, "w");
  if (!f) { fprintf(stderr, "cannot open %s\n", path); return 2; }
  fprintf(f, "{\n");
  fprintf(f, "  \"_note\": \"AUTO-GENERATED by tests/firmware/native/test_fault_machine.cpp "
             "--emit-golden. Firmware-anchored vectors: the REAL fault_machine.cpp drives them; "
             "tests/firmware/test_fault_logic.py replays them through controller/teensy_link/"
             "fault_logic.py so the host mirror cannot drift. Regenerate via "
             "tests/firmware/native/build.py --golden <path>.\",\n");

  // Error scenarios
  fprintf(f, "  \"error_scenarios\": [\n");
  auto escs = error_scenarios();
  for (size_t si = 0; si < escs.size(); ++si) {
    const auto& sc = escs[si];
    auto out = run_error_scenario(sc);
    fprintf(f, "    {\"name\": \"%s\", \"steps\": [\n", sc.name.c_str());
    for (size_t k = 0; k < sc.steps.size(); ++k) {
      const auto& st = sc.steps[k];
      fprintf(f, "      {\"notify_clear\": %s, \"preset_fatal\": %s, \"fatal_can_error\": %s, ",
              jb(st.notify_clear), jb(st.preset_fatal), jb(st.fatal_can_error));
      fprintf(f, "\"active\": "); emit_vec6u(f, st.active);
      fprintf(f, ", \"disarm\": "); emit_vec6u(f, st.disarm);
      fprintf(f, ", \"state\": "); emit_vec6b(f, st.state);
      fprintf(f, ", \"out\": {\"fatal_error\": %s, \"undervoltage_error\": %s, "
                 "\"soft_reset_attempts\": %d, \"clear_calls\": %d}}%s\n",
              jb(out[k].fatal), jb(out[k].uv), out[k].soft_reset, out[k].clear_calls,
              k + 1 < sc.steps.size() ? "," : "");
    }
    fprintf(f, "    ]}%s\n", si + 1 < escs.size() ? "," : "");
  }
  fprintf(f, "  ],\n");

  // Stow scenarios
  fprintf(f, "  \"stow_scenarios\": [\n");
  auto sscs = stow_scenarios();
  for (size_t si = 0; si < sscs.size(); ++si) {
    const auto& sc = sscs[si];
    auto out = run_stow_scenario(sc);
    fprintf(f, "    {\"name\": \"%s\", \"steps\": [\n", sc.name.c_str());
    for (size_t k = 0; k < sc.steps.size(); ++k) {
      const auto& st = sc.steps[k];
      fprintf(f, "      {\"stale\": %s, \"fresh\": %s, \"complete\": %s, "
                 "\"reboot_start\": %s, \"deadline_pass\": %s, \"seen\": %s, "
                 "\"out\": {\"fatal\": %s, \"stow_pending\": %s, \"stowing\": %s}}%s\n",
              jb(st.stale), jb(st.fresh), jb(st.complete), jb(st.reboot_start), jb(st.deadline_pass), jb(st.seen),
              jb(out[k].fatal), jb(out[k].pending), jb(out[k].stowing),
              k + 1 < sc.steps.size() ? "," : "");
    }
    fprintf(f, "    ]}%s\n", si + 1 < sscs.size() ? "," : "");
  }
  fprintf(f, "  ]\n}\n");
  fclose(f);
  return 0;
}

int main(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--emit-golden") == 0 && i + 1 < argc) {
      return emit_golden(argv[i + 1]);
    }
  }
  doctest::Context ctx;
  ctx.applyCommandLine(argc, argv);
  return ctx.run();
}
