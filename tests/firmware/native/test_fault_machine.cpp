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
//  subset of scenarios that teensy_link/fault_logic.py models
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
#include "leg_stow_gate.h"   // FW 53 U4: the CONFIRM phase's gate (linked leg_stow_gate.o)

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
    axes[i].last_rx_us = 0;
    axes[i].pos_rev = 0.0f;
    axes[i].vel_rps = 0.0f;
    axes[i].pos_timestamp_us = 0;
    axes[i].target_pos_rev = 0.0f;
    axes[i].target_vel_rps = 0.0f;
    axes[i].target_torque_Nm = 0.0f;
  }
}

// Stamp a heartbeat frame's ARRIVAL for axis `a` — BOTH timestamps, exactly as
// can_buses.cpp decode_into_cache does: the liveness stamp (last_rx_us, FW 23)
// is written above the per-command switch, so a heartbeat is also a liveness
// frame. Every existing scenario here goes through this helper, which is why the
// FW 23 predicate swap leaves the committed golden byte-identical: the two
// predicates AGREE whenever the only traffic on the bus is heartbeats. The tests
// that make them DISAGREE (a live encoder stream against a dead heartbeat, and
// the reverse) stamp the two fields independently and are the new coverage.
static void stamp_hb(uint8_t a) {
  axes[a].last_heartbeat_us = fake_mono_us();
  axes[a].last_rx_us        = fake_mono_us();
}
// Stamp ONLY the liveness clock — a non-heartbeat frame (encoder estimate, iq,
// temps, vbus, get_error). This is what the 2026-09-15 fault looked like from the
// bridge's side: ~272 frames/s/axis arriving with the 10 Hz heartbeat missing.
static void stamp_rx_only(uint8_t a) { axes[a].last_rx_us = fake_mono_us(); }
// Seed a PRESENT bus: heartbeat_seen latched (leg_present ⇒ the axis is on the
// bus) plus both clocks stamped. Presence is a SEPARATE latch from freshness and
// both predicates gate on it, so a test that stamps clocks without it is testing
// an absent axis, which is skipped by construction.
static void seed_present_legs() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i) { axes[i].heartbeat_seen = true; stamp_hb(i); }
}
static void seed_present_axes() {   // legs 0-5 + the hand (axis 6)
  for (uint8_t i = 0; i < NUM_AXES; ++i) { axes[i].heartbeat_seen = true; stamp_hb(i); }
}

static void reset_all() {
  fake_reset();
  leg_stow_gate_init();
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
    // s_fatal_can_error each tick (all_present_legs_alive is vacuously true with
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
// firmware's clock-driven any_present_leg_silent / all_present_legs_alive /
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

// In-window staleness advance: > CAN_AXIS_SILENCE_TIMEOUT_US (legs go stale) but
// < REBOOT_WATCHDOG_SUPPRESS_US (still inside the reboot-suppression window), so a
// "stale" step alone never crosses the reboot deadline — deadline_pass does that.
static constexpr uint64_t STALE_ADVANCE_US = CAN_AXIS_SILENCE_TIMEOUT_US + 500'000;  // 2.5 s

static std::vector<StowOut> run_stow_scenario(const StowScenario& sc) {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  if (!sc.cold_start) {
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {     // full robot: all 6 present
      axes[i].heartbeat_seen = true;
      stamp_hb(i);
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
        stamp_hb(i);
      }
    }
    if (st.reboot_start) fault_notify_reboot_started();               // arm the latch
    if (st.stale) fake_advance(STALE_ADVANCE_US);                     // legs go stale (in-window)
    if (st.deadline_pass) fake_advance(REBOOT_WATCHDOG_SUPPRESS_US);  // cross the reboot deadline
    if (st.fresh) for (uint8_t i = 0; i < NUM_LEGS; ++i) stamp_hb(i);
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
    stamp_hb(i);
  }
  // Loss: detection arms the latch and gates output OFF — and the fault machine
  // emits NOTHING to the dead bus (no clears: legs are error-free).
  fake_advance(3 * (uint64_t)CAN_AXIS_SILENCE_TIMEOUT_US);
  fake_clear_sent();
  fault_step();
  CHECK(s_fatal_can_error);
  CHECK(s_stow_pending);
  CHECK(interp_output_enabled() == false);     // never command the dead bus (gate)
  CHECK(fake_sent_count() == 0);                // ...and nothing was sent to it
  CHECK(fault_state() == JbUdp::FaultState::CAN_BUS_DOWN);

  // Confirmed reconnect: the profiled stow begins, output gate re-enabled.
  for (uint8_t i = 0; i < NUM_LEGS; ++i) stamp_hb(i);
  fault_step();
  CHECK(s_fatal_can_error == false);
  CHECK(s_stowing);
  CHECK(interp_stow_active());
  CHECK(interp_output_enabled());

  // Drive the descent to completion (legs already at the off pose), then the
  // fault machine IDLEs every leg and clears the latch. FW 53: the IDLE needs the
  // ENCODERS at Stow (CONFIRM), so the six Legs report Closed-loop at 0 rev, at
  // rest, in fresh heartbeat + encoder frames.
  interp_isr();                                 // one tick → stow_complete
  CHECK(interp_stow_complete());
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    axes[i].axis_state = ST_CL;
    stamp_hb(i);
    write_pos_vel(axes[i], 0.0f, 0.0f, fake_mono_us());
  }
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
  stamp_hb(0);
  // Loss: leg 0 goes stale → detection fires even with five absent legs.
  fake_advance(3 * (uint64_t)CAN_AXIS_SILENCE_TIMEOUT_US);
  fault_step();
  CHECK(s_fatal_can_error);
  CHECK(s_stow_pending);
  // Reconnect: leg 0 fresh again → stow EXECUTES (the dead-lock fix).
  stamp_hb(0);
  fault_step();
  CHECK(s_fatal_can_error == false);
  CHECK(s_stowing);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  FW 23 — the AXIS-SILENCE watchdog (the predicate swap)
// ═══════════════════════════════════════════════════════════════════════════════
//  WHY THESE EXIST. Until FW 23 the fatal CAN_BUS_DOWN predicate read
//  last_heartbeat_us, i.e. the ONE stream an ODrive sends at 10 Hz out of the
//  ~272 frames/s it broadcasts. On 2026-09-15 that stowed the robot twice while
//  the bus was demonstrably healthy: 0 wire errors, ring leak 0, ~1950 frames/s
//  RX, worst per-axis ENCODER age 95 ms. An ODrive DROPS (never queues) a cyclic
//  frame that finds no free TX mailbox, so under 62 % bus load the sparsest
//  stream is the one that produces a 2 s run of consecutive losses. The predicate
//  now reads last_rx_us (ANY frame from that node), and these cases pin the two
//  directions in which the old and new predicates DISAGREE — which is precisely
//  the behaviour no committed golden could have caught, since every golden
//  scenario's only bus traffic is heartbeats (stamp_hb stamps both clocks).
TEST_CASE("FW 23: a leg with a DEAD heartbeat but a live frame stream does NOT trip") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  seed_present_legs();
  fault_step();
  REQUIRE(fault_can_bus_down() == false);

  // Replay the 2026-09-15 shape on leg 3: its 10 Hz heartbeat stops dead while
  // its 100 Hz encoder stream keeps arriving. Walk 6 s in 100 ms steps, stamping
  // liveness only — three times the 2.0 s window, so the OLD predicate would
  // have tripped ~20 times over.
  for (int k = 0; k < 60; ++k) {
    fake_advance(100'000);
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      if (i == 3) stamp_rx_only(i);   // frames flowing, heartbeat silent
      else        stamp_hb(i);
    }
    fault_step();
    REQUIRE(fault_can_bus_down() == false);
    REQUIRE(fault_stow_pending() == false);
  }
  // No trip, no stow, nothing latched — but the dropout IS reported.
  REQUIRE(fault_can_fault_leg() == 0xFF);
  REQUIRE(fault_can_fault_count() == 0);
  REQUIRE((fault_hb_stale_mask() & (1u << 3)) != 0);   // reported as a diagnostic
  REQUIRE((fault_hb_stale_mask() & ~(1u << 3)) == 0);  // and ONLY on leg 3
}

TEST_CASE("FW 23: a leg silent on EVERY stream DOES trip, and the latch names it") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  seed_present_legs();
  fault_step();
  REQUIRE(fault_can_bus_down() == false);
  REQUIRE(fault_can_fault_leg() == 0xFF);   // 0xFF ⇒ no trip since boot

  // Leg 4 goes completely dark; the other five keep streaming. One silent leg is
  // an ACTUATOR LOSS on a closed kinematic chain, so a single leg still trips.
  fake_advance(CAN_AXIS_SILENCE_TIMEOUT_US + 300'000);   // 2.3 s
  for (uint8_t i = 0; i < NUM_LEGS; ++i) if (i != 4) stamp_hb(i);
  fault_step();
  REQUIRE(fault_can_bus_down() == true);
  REQUIRE(fault_stow_pending() == true);
  REQUIRE(fault_can_fault_leg() == 4);
  // The age is the SILENCE age, frozen at the trip: > the 2000 ms threshold by
  // construction, and near it for a marginal dropout rather than a bus loss.
  REQUIRE(fault_can_fault_age_ms() >= 2300);
  REQUIRE(fault_can_fault_age_ms() < 2400);
  REQUIRE(fault_can_fault_count() == 1);
}

TEST_CASE("FW 23: the trip latch SURVIVES the self-clear, and counts every trip") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  seed_present_legs();
  fault_step();

  // Trip on leg 2.
  fake_advance(CAN_AXIS_SILENCE_TIMEOUT_US + 100'000);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) if (i != 2) stamp_hb(i);
  fault_step();
  REQUIRE(fault_can_bus_down() == true);
  REQUIRE(fault_can_fault_leg() == 2);
  REQUIRE(fault_can_fault_count() == 1);

  // Leg 2 comes back → the fault SELF-CLEARS (the 2026-09-15 events cleared
  // inside one 10 Hz tick). The latch must NOT clear with it: it is the only
  // record the operator or a bag reader ever gets.
  seed_present_legs();
  fault_step();
  REQUIRE(fault_can_bus_down() == false);
  REQUIRE(fault_can_fault_leg() == 2);
  REQUIRE(fault_can_fault_count() == 1);

  // A second, different trip re-points the latch and advances the census.
  fake_advance(CAN_AXIS_SILENCE_TIMEOUT_US + 100'000);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) if (i != 5) stamp_hb(i);
  fault_step();
  REQUIRE(fault_can_fault_leg() == 5);
  REQUIRE(fault_can_fault_count() == 2);

  // An operator CLEAR_ERRORS releases the identity (acknowledged) but KEEPS the
  // count: a session census an operator clear could zero would make the second
  // trip of a sitting read as the first.
  fault_notify_clear_errors();
  REQUIRE(fault_can_fault_leg() == 0xFF);
  REQUIRE(fault_can_fault_age_ms() == 0);
  REQUIRE(fault_can_fault_count() == 2);
}

TEST_CASE("FW 23: whole-bus darkness is the ALL-legs instance of the same predicate") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  seed_present_legs();
  fault_step();
  REQUIRE(fault_can_bus_down() == false);

  // Nothing stamped at all — the cable-unplug case the fault is named for.
  fake_advance(CAN_AXIS_SILENCE_TIMEOUT_US + 500'000);
  fault_step();
  REQUIRE(fault_can_bus_down() == true);
  REQUIRE(fault_stow_pending() == true);
  REQUIRE(fault_can_fault_leg() == 0);      // every leg silent ⇒ the FIRST in index order
  REQUIRE(fault_can_fault_count() == 1);
}

TEST_CASE("FW 23: HB_STALE_MASK is per-axis, includes the hand, and is 4x sharper than the trip") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  REQUIRE(fault_hb_stale_mask() == 0);      // nothing seen yet ⇒ no axis is "stale"

  seed_present_axes();
  REQUIRE(fault_hb_stale_mask() == 0);

  // Just INSIDE the 0.5 s diagnostic window: leg 1 and the hand stop heartbeating
  // (their frame streams keep flowing, so nothing may trip).
  fake_advance(400'000);
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    if (i == 1 || i == HAND_AXIS) stamp_rx_only(i);
    else                          stamp_hb(i);
  }
  fault_step();
  REQUIRE(fault_hb_stale_mask() == 0);      // 400 ms < CAN_HEARTBEAT_STALE_US
  REQUIRE(fault_can_bus_down() == false);

  // Past it, but still far short of the 2.0 s silence timeout.
  fake_advance(200'000);                    // those two axes now 600 ms stale
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    if (i == 1 || i == HAND_AXIS) stamp_rx_only(i);
    else                          stamp_hb(i);
  }
  fault_step();
  REQUIRE(fault_hb_stale_mask() == ((1u << 1) | (1u << HAND_AXIS)));
  REQUIRE(fault_can_bus_down() == false);   // diagnostic ONLY — never a fault
  REQUIRE(fault_stow_pending() == false);
  REQUIRE(fault_can_fault_leg() == 0xFF);

  // The mask fits the 7 bits the wire reserves for it (flags bits 16-22).
  REQUIRE((fault_hb_stale_mask() & ~0x7Fu) == 0);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  FW 46 (BUG-93) — a receive stamp NEWER than the clock read is a LIVE axis
// ═══════════════════════════════════════════════════════════════════════════════
//  On the board, every age test captures `now = micros64()` and then reads the
//  axis timestamp; task_can_rx (priority 5) can preempt the fault task
//  (priority 3) between the two and stamp a frame that is newer than `now`.
//  Until FW 46 the unsigned `now - ts` then wrapped to ~2^64 and a live leg read
//  as silent for one evaluation: a CAN_BUS_DOWN trip and a Stow mid-Attempt,
//  latched as leg 0xFF / age 0 (the latch re-reads the clock after the writer is
//  done). The native harness has no preemption, but the interleaving is fully
//  reproduced by stamping ts > fake_mono_us(): the predicate reads exactly what
//  it would have read on the board. Recipe confirmed against the unguarded code:
//  3 of the 33 doctest cases fail there (a case stops at its first failing
//  REQUIRE, so not every [wraps] REQUIRE was observed failing; logbook
//  2026-11-15-canbridge-silence-predicate-wrap).
static void stamp_future(uint8_t a, uint64_t ahead_us) {
  axes[a].last_rx_us        = fake_mono_us() + ahead_us;
  axes[a].last_heartbeat_us = fake_mono_us() + ahead_us;
}

TEST_CASE("FW 46 BUG-93: a stamp after the clock read is ALIVE, never SILENT") {
  for (uint64_t ahead : {uint64_t{1}, uint64_t{3'600'000'000ULL}}) {   // 1 us, 1 h
    CAPTURE(ahead);
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    seed_present_axes();
    fault_step();
    REQUIRE(fault_can_bus_down() == false);

    // Every axis stamped AFTER `now` — the worst case: the whole bus in the window.
    for (uint8_t i = 0; i < NUM_AXES; ++i) stamp_future(i, ahead);
    REQUIRE(any_present_leg_silent() == false);   // [wraps]
    REQUIRE(all_present_legs_alive() == true);    // [wraps]
    uint32_t age_ms = 12345;
    REQUIRE(first_silent_leg(&age_ms) == 0xFF);   // [wraps]
    REQUIRE(age_ms == 0);
    REQUIRE(fault_hb_stale_mask() == 0);          // [wraps]

    fault_step();
    REQUIRE(fault_can_bus_down() == false);       // [wraps] no CAN_BUS_DOWN trip
    REQUIRE(fault_stow_pending() == false);
    REQUIRE(fault_can_fault_leg() == 0xFF);
    REQUIRE(fault_can_fault_count() == 0);
    for (uint8_t i = 0; i < NUM_AXES; ++i) {
      CAPTURE(i);
      REQUIRE(axes[i].heartbeat_stale == false);  // [wraps] telemetry bookkeeping too
    }
  }
}

TEST_CASE("FW 46 BUG-93: a future stamp on one leg neither hides nor steals a real silence") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  seed_present_legs();
  fault_step();

  // Leg 3 genuinely silent by one microsecond past the window; leg 0 stamped in
  // the preemption window; the rest fresh at `now`.
  fake_advance(CAN_AXIS_SILENCE_TIMEOUT_US + 1);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) if (i != 3) stamp_hb(i);
  stamp_future(0, 1);
  REQUIRE(any_present_leg_silent() == true);
  REQUIRE(all_present_legs_alive() == false);
  uint32_t age_ms = 0;
  REQUIRE(first_silent_leg(&age_ms) == 3);        // [wraps] unguarded names leg 0
  REQUIRE(age_ms == (CAN_AXIS_SILENCE_TIMEOUT_US + 1) / 1000);

  fault_step();
  REQUIRE(fault_can_bus_down() == true);          // the timeout still trips
  REQUIRE(fault_stow_pending() == true);
  REQUIRE(fault_can_fault_leg() == 3);            // [wraps]
  REQUIRE(fault_can_fault_count() == 1);

  // Reconnect inside the window: every leg's next frame lands after `now`. The
  // inverted predicate must read that as alive and self-clear on this tick.
  for (uint8_t i = 0; i < NUM_LEGS; ++i) stamp_future(i, 1);
  fault_step();
  REQUIRE(fault_can_bus_down() == false);         // [wraps]
}

TEST_CASE("FW 46 BUG-93: the silence boundary is unchanged (timeout trips only past it)") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  seed_present_legs();
  fault_step();

  fake_advance(CAN_AXIS_SILENCE_TIMEOUT_US);      // age == timeout: not yet silent
  for (uint8_t i = 0; i < NUM_LEGS; ++i) if (i != 2) stamp_hb(i);
  fault_step();
  REQUIRE(fault_can_bus_down() == false);

  fake_advance(1);                                // age == timeout + 1: silent
  for (uint8_t i = 0; i < NUM_LEGS; ++i) if (i != 2) stamp_hb(i);
  fault_step();
  REQUIRE(fault_can_bus_down() == true);
  REQUIRE(fault_can_fault_leg() == 2);
}

TEST_CASE("FW 46 BUG-93: rx_older_than is strict on both sides") {
  REQUIRE(rx_older_than(100, 100, 0) == false);   // same instant: age zero
  REQUIRE(rx_older_than(100, 101, 0) == false);   // future stamp: age zero, not 2^64
  REQUIRE(rx_older_than(100, ~0ULL, 0) == false);
  REQUIRE(rx_older_than(100, 99, 0) == true);
  REQUIRE(rx_older_than(100, 50, 50) == false);   // age == limit: not older
  REQUIRE(rx_older_than(100, 49, 50) == true);
}

TEST_CASE("motor-feedback staleness suppresses output recoverably (not latched)") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fault_set_mpc_active(true);
  axes[0].heartbeat_seen = true;
  stamp_hb(0);
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
  stamp_hb(0);   // bus still up
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
    stamp_hb(0);
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

  SUBCASE("present leg diverging trips MAX_DEVIATION + freezes the latch snapshot") {
    arm_leg0();
    latch_u0(1.2f, 3.0f);             // leg0 (present) diverges 1.2 > 1.0 (MAX_DEVIATION_REV, raised 0.5→1.0 on 2026-07-16); leg1 (absent) far
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::MAX_DEVIATION);
    // 2026-07-10 latch-event snapshot: which leg crossed + dev/u0/encoder at the trip.
    CHECK(fault_max_dev_leg() == 0);
    CHECK(fault_max_dev_value() == doctest::Approx(1.2f));
    CHECK(fault_max_dev_u0() == doctest::Approx(1.2f));
    CHECK(fault_max_dev_enc() == doctest::Approx(0.0f));
  }
  SUBCASE("absent leg's far command alone does NOT trip MAX_DEVIATION") {
    arm_leg0();
    latch_u0(0.0f, 3.0f);             // leg0 aligned; only the absent leg1 is far
    fault_step();
    CHECK(fault_state() != JbUdp::FaultState::MAX_DEVIATION);
    CHECK(fault_max_dev_leg() == 0xFF);   // no MAX_DEVIATION latch → snapshot unset
  }
  SUBCASE("fb-stale does not false-trip pre-arm (mpc inactive)") {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    fault_set_mpc_active(false);      // not armed
    axes[0].heartbeat_seen = true;
    stamp_hb(0);
    axes[0].pos_timestamp_us = 0;     // ancient feedback timestamp
    fake_advance(1'000'000);
    stamp_hb(0);
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
    stamp_hb(0);
    axes[0].pos_rev = 0.0f;
    axes[0].pos_timestamp_us = fake_mono_us();
    fake_set_udp_last_rx_us(fake_mono_us());
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
    interp_on_setpoint(0, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
    fake_set_udp_last_rx_us(fake_mono_us());   // keep the link fresh after the tick
  };

  // ── INVARIANTS Gap 5: the setpoint-staleness E-STOP had no compiled test ─────
  //  The 40 Hz Setpoint stream dying mid-flight is the hard link-fault trigger —
  //  the one that must stop a moving robot when the Jetson, the network or the
  //  trajectory node goes away. Recipe (all four freshness clocks are separate,
  //  and only ONE is allowed to age or the assert proves the wrong thing):
  //    arm() stamps interp_last_setpoint_us at mono = 10.000 s, then
  //    fake_advance(SETPOINT_STALENESS_US + 1) ages the MONO clock past 0.25 s
  //    while last_heartbeat_us / pos_timestamp_us / udp_last_rx_us are re-stamped
  //    to the NEW now — so the CAN watchdog, MOTOR_FB_STALE and the link check all
  //    stay satisfied and SETPOINT_STALE is the only reachable verdict.
  //  `ever_cmd` (interp_last_setpoint_us() != 0) is why arm()'s setpoint matters:
  //  a board that has never been commanded must not E-STOP on boot.
  SUBCASE("setpoint staleness E-STOPs and LATCHES until an explicit clear (Gap 5)") {
    arm();
    fault_step();
    REQUIRE(fault_state() == JbUdp::FaultState::NONE);      // fresh stream, no trip

    fake_advance(SETPOINT_STALENESS_US + 1);                // ONLY the setpoint ages
    stamp_hb(0);
    axes[0].pos_timestamp_us  = fake_mono_us();
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::SETPOINT_STALE);
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    CHECK(interp_output_enabled() == false);

    // A fresh setpoint does NOT release it — unlike MOTOR_FB_STALE (recoverable),
    // this is a LATCHED guard E-STOP: the stream returning is not the operator
    // saying it is safe to move again.
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
    interp_on_setpoint(0, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
    stamp_hb(0);
    axes[0].pos_timestamp_us  = fake_mono_us();
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);   // still latched

    // Only an explicit operator CLEAR_ERRORS releases it.
    fault_notify_clear_errors();
    fault_step();
    CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);
  }

  // FW 52 (BUG-99 D2): a PER-AXIS clear acknowledges one axis, not the robot. It
  // must leave the Soft E-stop latched; only the all-axes clear releases it.
  SUBCASE("FW 52 D2: a per-axis clear does NOT release the latched Soft E-stop; the all-axes clear does") {
    arm();
    fault_step();
    REQUIRE(fault_state() == JbUdp::FaultState::NONE);
    fake_advance(SETPOINT_STALENESS_US + 1);
    stamp_hb(0);
    axes[0].pos_timestamp_us  = fake_mono_us();
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    REQUIRE(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    // The stream returns (as in the Gap 5 subcase above), so the CONDITION is gone and
    // only the latch holds the E-stop: exactly what a per-axis clear must not release.
    // (fw52-verify 2026-11-25: the first draft cleared with the setpoint still stale,
    // so the all-axes clear re-tripped at once; a test defect, not a firmware one.)
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
    interp_on_setpoint(0, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
    stamp_hb(0);
    axes[0].pos_timestamp_us  = fake_mono_us();
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    REQUIRE(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    axes[3].disarm_reason = 0x40u;
    fault_notify_clear_axis_errors(3);
    CHECK(axes[3].disarm_reason == 0u);           // that axis's cache is cleared
    fault_notify_clear_axis_errors(0);
    fault_step();
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);   // still latched
    CHECK(fault_state() == JbUdp::FaultState::SETPOINT_STALE);
    fault_notify_clear_axis_errors(200);           // out of range: a no-op, no crash
    fault_notify_clear_errors();
    fault_step();
    CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);
  }

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
    sp.u0[0] = 1.2f;                              // leg0 base diverges 1.2 > MAX_DEVIATION_REV (1.0, raised 0.5→1.0 on 2026-07-16)
    interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    REQUIRE(fault_state() == JbUdp::FaultState::MAX_DEVIATION);
    REQUIRE(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    // Shrink the divergence (encoder catches up). Latch must hold.
    axes[0].pos_rev = 1.2f;
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
  for (uint8_t i = 0; i < NUM_LEGS; ++i) { axes[i].heartbeat_seen = true; stamp_hb(i); }
  // NO fault_notify_reboot_started().
  fake_advance(3 * (uint64_t)CAN_AXIS_SILENCE_TIMEOUT_US);
  fault_step();
  CHECK(fault_can_bus_down());                              // detection fired
  CHECK(fault_stow_pending());                             // deferred stow armed (inversion intact)
  CHECK(fault_state() == JbUdp::FaultState::CAN_BUS_DOWN);
}

TEST_CASE("reboot latch: suppresses in-window, releases on fresh-after-stale AND on deadline") {
  // (a) fresh-after-stale release — the ODrives came back.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) { axes[i].heartbeat_seen = true; stamp_hb(i); }
  fault_notify_reboot_started();                            // arm (legs fresh at arm time)
  fake_advance(CAN_AXIS_SILENCE_TIMEOUT_US + 500'000);         // 2.5 s: legs stale, < 6 s deadline
  fault_step();
  CHECK(fault_can_bus_down() == false);                    // SUPPRESSED — no false loss
  CHECK(fault_stow_pending() == false);                    // ...and no stow armed
  for (uint8_t i = 0; i < NUM_LEGS; ++i) stamp_hb(i);  // legs return
  fault_step();
  CHECK(fault_can_bus_down() == false);                    // released cleanly, no stow
  CHECK(fault_stow_pending() == false);

  // (b) deadline release — a reboot that never returns is still caught at the deadline.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  for (uint8_t i = 0; i < NUM_LEGS; ++i) { axes[i].heartbeat_seen = true; stamp_hb(i); }
  fault_notify_reboot_started();
  fake_advance(CAN_AXIS_SILENCE_TIMEOUT_US + 500'000);         // 2.5 s: in-window → suppressed
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
  for (uint8_t i = 0; i < NUM_LEGS; ++i) { axes[i].heartbeat_seen = true; stamp_hb(i); }
  fault_notify_reboot_started();                            // arm (legs fresh)
  fault_step();                                             // legs still fresh → must NOT release
  fake_advance(CAN_AXIS_SILENCE_TIMEOUT_US + 500'000);         // now the reboot silence arrives (in-window)
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
      stamp_hb(i);
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
    fake_advance(3 * (uint64_t)CAN_AXIS_SILENCE_TIMEOUT_US);   // both clocks move → real staleness on mono
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
    stamp_hb(HAND_AXIS);
    fake_advance(3 * (uint64_t)CAN_AXIS_SILENCE_TIMEOUT_US);   // hand heartbeat goes stale
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
             "tests/firmware/test_fault_logic.py replays them through teensy_link/"
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


// ═══════════════════════════════════════════════════════════════════════════
//  FW 17 — hand guards join the fault machine with their OWN constants (T-U9)
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("hand overspeed guards at HAND_MAX_MOTOR_VEL_RPS (345), never the legs' 16.5") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fake_set_udp_last_rx_us(fake_mono_us());

  // A legacy stroke's certified peak region (178–221 rev/s) sails through: the
  // leg bound applied to axis 6 would E-STOP every hand move ever made.
  axes[HAND_AXIS].vel_rps = 250.0f;
  fault_step();
  CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);
  CHECK(fault_state() != JbUdp::FaultState::MOTOR_OVERSPEED);

  // Past the hand's own bound (1.15 × the 300 rev/s ceiling) → E-STOP.
  axes[HAND_AXIS].vel_rps = HAND_MAX_MOTOR_VEL_RPS + 5.0f;
  fault_step();
  CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
  CHECK(fault_state() == JbUdp::FaultState::MOTOR_OVERSPEED);

  // And a LEG at hand speeds still trips its own 16.5 bound (separation is
  // two-sided — widening the leg loop must not have loosened the legs).
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fake_set_udp_last_rx_us(fake_mono_us());
  axes[2].vel_rps = MAX_MOTOR_VEL_RPS + 1.0f;
  fault_step();
  CHECK(fault_state() == JbUdp::FaultState::MOTOR_OVERSPEED);
}

TEST_CASE("hand deviation: observe-first reports only; `hand7 arm`ed it LATCHES off the 500 Hz tick verdict") {
  // The full chain: STREAMED via the real settle gate → a HAS_HAND frame whose
  // command sits 3.0 rev from the (age-extrapolated) encoder → the interp tick
  // counts the exceed → the 10 Hz fault task differences the counter and, only
  // when armed, latches MAX_DEVIATION with the axis-6 snapshot.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fault_set_mpc_active(true);
  fake_set_udp_last_rx_us(fake_mono_us());
  // leg 0 present + fresh so the leg-side guards are quiet and present-scoped.
  axes[0].heartbeat_seen = true;
  stamp_hb(0);
  axes[0].pos_timestamp_us = fake_mono_us();
  // Axis 6 fresh on the encoder — the I-HAND-5 precondition for the lane.
  axes[HAND_AXIS].heartbeat_seen = true;
  stamp_hb(HAND_AXIS);
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());

  // Latch a v6 frame: legs at their encoders (dev 0), hand commanded 3.0 rev
  // from its encoder (> MAX_DEVIATION_HAND_REV 2.5).
  JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
  sp.u0[HAND_AXIS] = 3.0f;
  sp.flags = 0x4u;                                   // HAS_HAND
  interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
  interp_isr();
  // FW 18 gate: output is still SUPPRESSED on this tick (the first fault_step
  // below is what enables it), so the tick computes the residual and reports
  // it — dev_max/dev_last are ungated observations — but counts NOTHING. Pre
  // FW 18 it counted a clamp on a frame that never reached the wire.
  CHECK(interp_hand_dev_over_ticks() == 0);           // suppressed ⇒ uncounted
  fake_set_udp_last_rx_us(fake_mono_us());

  // OBSERVE — forced explicitly, since the guard boots ARMED since FW 21: the
  // verdict is reported, never latched. (Boot ARMED is pinned by the leg_interp
  // disarm-edge test; this block proves observe SUPPRESSES a genuine exceed.)
  interp_set_hand_dev_guard_armed(false);
  fault_step();
  CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);
  CHECK(interp_hand_dev_max() == doctest::Approx(3.0f).epsilon(0.01));

  // ARMED (the second sitting's explicit step): the next exceed tick latches.
  // FIX-1 interplay (2026-09-02): the fault_step above ENABLED output, so the
  // next tick is an output-enable (arm) edge and CLEARS the hand-lane latch —
  // exactly as a real armed session begins. The live 40 Hz stream re-latches
  // with its next HAS_HAND frame; model that with a fresh frame here.
  interp_set_hand_dev_guard_armed(true);
  fake_advance(INTERP_PERIOD_US);
  JbUdp::SetpointPayload sp2; memset(&sp2, 0, sizeof(sp2));
  sp2.u0[HAND_AXIS] = 3.0f;
  sp2.flags = 0x4u;                                  // HAS_HAND
  interp_on_setpoint(2, reinterpret_cast<const uint8_t*>(&sp2), sizeof(sp2));
  interp_isr();                                      // another exceed tick
  CHECK(interp_hand_dev_over_ticks() == 1);          // the first counted tick
  fake_set_udp_last_rx_us(fake_mono_us());
  fault_step();
  CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
  CHECK(fault_state() == JbUdp::FaultState::MAX_DEVIATION);
  CHECK(fault_max_dev_leg() == HAND_AXIS);           // the snapshot names axis 6
  CHECK(fault_max_dev_value() == doctest::Approx(3.0f).epsilon(0.01));
  CHECK(fault_max_dev_u0() == doctest::Approx(3.0f).epsilon(0.01));
  CHECK(fault_max_dev_enc() == doctest::Approx(0.0f).epsilon(0.01));
  CHECK(interp_output_enabled() == false);

  // CLEAR_ERRORS releases the latch; with the condition still live (the lane
  // keeps commanding 3.0 against a 0.0 encoder) the next tick+poll re-latches —
  // the leg guard's cannot-clear-through-a-live-condition contract, inherited.
  fault_notify_clear_errors();
  fake_set_udp_last_rx_us(fake_mono_us());
  fault_step();                                      // the clear re-ENABLES output
  // FW 18: re-latching now takes one more poll than it did, and that is the
  // gate working. While output was suppressed the lane commanded nothing, so
  // its ticks count nothing; the trip can only be re-armed by a tick that
  // actually transmits. The clear's own output-enable is an arm edge, so the
  // lane needs a fresh HAS_HAND frame exactly as a live 40 Hz stream provides.
  fake_advance(INTERP_PERIOD_US);
  JbUdp::SetpointPayload sp3; memset(&sp3, 0, sizeof(sp3));
  sp3.u0[HAND_AXIS] = 3.0f;
  sp3.flags = 0x4u;                                  // HAS_HAND
  interp_on_setpoint(3, reinterpret_cast<const uint8_t*>(&sp3), sizeof(sp3));
  interp_isr();                                      // transmitting ⇒ counted
  fake_set_udp_last_rx_us(fake_mono_us());
  fault_step();
  CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
  CHECK(fault_state() == JbUdp::FaultState::MAX_DEVIATION);
}

TEST_CASE("`hand7 reset` with a banked baseline and the latch already cleared does not re-trip (A-N1)") {
  // Reaches the exact state the fix is guarding: s_hand_dev_over_prev banked to
  // 1 by a prior exceed tick, the ESTOP already released by CLEAR_ERRORS, guard
  // still ARMED. `hand7 reset` must leave the guard quiet — it always did here
  // (this is the ORIGINAL masking-safe direction: a reset that only zeroes
  // going forward). The real regression case for the one-poll masking window
  // this fix closes is the NEXT test.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fault_set_mpc_active(true);
  fake_set_udp_last_rx_us(fake_mono_us());
  axes[0].heartbeat_seen = true;
  stamp_hb(0);
  axes[0].pos_timestamp_us = fake_mono_us();
  axes[HAND_AXIS].heartbeat_seen = true;
  stamp_hb(HAND_AXIS);
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
  interp_set_hand_dev_guard_armed(true);

  // Enable output (a quiet first poll), then transmit ONE exceeding frame so
  // s_hand_dev_over_prev banks to 1 and the guard latches.
  fault_step();
  JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
  sp.u0[HAND_AXIS] = 3.0f;
  sp.flags = 0x4u;                                    // HAS_HAND
  interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
  interp_isr();
  CHECK(interp_hand_dev_over_ticks() == 1);
  fake_set_udp_last_rx_us(fake_mono_us());
  fault_step();
  CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);

  // Release the latch — the condition itself is not re-driven (no fresh
  // frame), so this fault_step re-enables output and leaves the guard quiet:
  // exactly "banked s_hand_dev_over_prev == 1 with the latch cleared".
  fault_notify_clear_errors();
  fake_set_udp_last_rx_us(fake_mono_us());
  fault_step();
  CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);

  CHECK(interp_hand7_console("hand7 reset"));
  fake_set_udp_last_rx_us(fake_mono_us());
  fault_step();
  CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);
  CHECK(fault_state() != JbUdp::FaultState::MAX_DEVIATION);
}

TEST_CASE("`hand7 reset` re-baselines BOTH sides of the `>` comparison — closes the one-poll masking window (A-N1)") {
  // THE regression case: without fault_hand_dev_prev_reset(), a `hand7 reset`
  // zeroes leg_interp.cpp's s_hand_dev_over_ticks but NOT fault_machine.cpp's
  // own s_hand_dev_over_prev baseline. Bank prev = 5 via five exceed ticks,
  // reset, then drive exactly 3 FRESH exceed ticks before the next poll: post
  // reset the counter reads 3, and 3 > 5 is false, so a genuine, armed
  // exceedance would be silently swallowed for one whole 10 Hz poll window.
  // The fix (fault_hand_dev_prev_reset() called from
  // interp_hand_counters_reset()) re-baselines s_hand_dev_over_prev to 0 at
  // the same instant, so 3 > 0 is true and the trip fires as it must.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fault_set_mpc_active(true);
  fake_set_udp_last_rx_us(fake_mono_us());
  axes[0].heartbeat_seen = true;
  stamp_hb(0);
  axes[0].pos_timestamp_us = fake_mono_us();
  axes[HAND_AXIS].heartbeat_seen = true;
  stamp_hb(HAND_AXIS);
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
  interp_set_hand_dev_guard_armed(true);
  fault_step();                                       // enable output (quiet)

  auto exceed_tick = [&](uint16_t seq) {
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
    sp.u0[HAND_AXIS] = 3.0f;
    sp.flags = 0x4u;                                  // HAS_HAND
    interp_on_setpoint(seq, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
    fake_advance(INTERP_PERIOD_US);
  };

  // Five exceed ticks, banked into s_hand_dev_over_prev by a poll BEFORE the
  // reset — CLEAR_ERRORS after each latch so the loop can keep ticking without
  // the guard blocking output.
  uint16_t seq = 1;
  for (int i = 0; i < 5; ++i) {
    exceed_tick(seq++);
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    if (fault_guard_mode() == JbUdp::GuardMode::ESTOP) {
      fault_notify_clear_errors();
      fake_set_udp_last_rx_us(fake_mono_us());
      fault_step();                                   // re-enable output
    }
  }
  CHECK(interp_hand_dev_over_ticks() == 5);

  CHECK(interp_hand7_console("hand7 reset"));
  CHECK(interp_hand_dev_over_ticks() == 0);

  // Exactly 3 fresh exceed ticks post-reset — fewer than the pre-reset baseline
  // of 5. Pre-fix this would NOT trip (3 > 5 is false); post-fix it must,
  // because the baseline reset to 0 alongside the counter.
  for (int i = 0; i < 3; ++i) exceed_tick(seq++);
  CHECK(interp_hand_dev_over_ticks() == 3);
  fake_set_udp_last_rx_us(fake_mono_us());
  fault_step();
  CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
  CHECK(fault_state() == JbUdp::FaultState::MAX_DEVIATION);
}

TEST_CASE("hand MAX_DEVIATION latch reports the TRIP's excursion, never the boot-cumulative max (2026-09-02 fix)") {
  // An observe-block excursion (4.0 rev) precedes an ARMED trip whose own
  // excursion is smaller (2.8 rev). Pre-fix, the latch froze the OLD 4.0 rev
  // residual + its cmd/fb pair (interp_hand_dev_max) into max_dev_value/u0/enc,
  // misattributing the trip in /link_status and bags. Post-fix the latch
  // carries the exceed tick that fired THIS trip; the console's observe-first
  // dev_max read is unchanged.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fault_set_mpc_active(true);
  fake_set_udp_last_rx_us(fake_mono_us());
  axes[0].heartbeat_seen = true;
  stamp_hb(0);
  axes[0].pos_timestamp_us = fake_mono_us();
  axes[HAND_AXIS].heartbeat_seen = true;
  stamp_hb(HAND_AXIS);
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());

  // OBSERVE block: a 4.0 rev excursion becomes the boot-cumulative max.
  JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
  sp.u0[HAND_AXIS] = 4.0f;
  sp.flags = 0x4u;                                   // HAS_HAND
  interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
  interp_isr();
  // FW 18 gate: output is still SUPPRESSED on this tick (the first fault_step
  // below is what enables it), so the tick computes the residual and reports
  // it — dev_max/dev_last are ungated observations — but counts NOTHING. Pre
  // FW 18 it counted a clamp on a frame that never reached the wire.
  CHECK(interp_hand_dev_over_ticks() == 0);           // suppressed ⇒ uncounted
  CHECK(interp_hand_dev_max() == doctest::Approx(4.0f).epsilon(0.01));
  fault_step();                                      // observe: reported, never latched
  CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);

  // ARMED: a SMALLER exceed (2.8 > 2.5) fires the trip. (The fault_step above
  // enabled output, so this tick is an arm edge — the fresh HAS_HAND frame
  // re-latches the lane after the fix-1 edge clear, as the live stream would.)
  interp_set_hand_dev_guard_armed(true);
  fake_advance(INTERP_PERIOD_US);
  JbUdp::SetpointPayload sp2; memset(&sp2, 0, sizeof(sp2));
  sp2.u0[HAND_AXIS] = 2.8f;
  sp2.flags = 0x4u;
  interp_on_setpoint(2, reinterpret_cast<const uint8_t*>(&sp2), sizeof(sp2));
  interp_isr();
  CHECK(interp_hand_dev_over_ticks() == 1);          // the first counted tick
  fake_set_udp_last_rx_us(fake_mono_us());
  fault_step();
  CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
  CHECK(fault_state() == JbUdp::FaultState::MAX_DEVIATION);
  CHECK(fault_max_dev_leg() == HAND_AXIS);
  CHECK(fault_max_dev_value() == doctest::Approx(2.8f).epsilon(0.01));   // the TRIP's excursion
  CHECK(fault_max_dev_u0() == doctest::Approx(2.8f).epsilon(0.01));      // its own cmd …
  CHECK(fault_max_dev_enc() == doctest::Approx(0.0f).epsilon(0.01));     // … and fb pair
  CHECK(interp_hand_dev_max() == doctest::Approx(4.0f).epsilon(0.01));   // console read unchanged
}

// ═══ FW 27: the hand guard reads only FRESH feedback ═════════════════════════
//
//  R5 sitting 4 attempt 2 (2026-10-04 1791104246.0) latched MAX_DEVIATION on
//  axis 6 with u0 9.6116 against fb_ex 6.3405: the anchor was 99.9 ms old,
//  frozen at 1.213 rev / 51.30 rev/s mid-stroke by an ODrive-side encoder frame
//  drop (plans/active/leg-bus-frame-drops.md), so fb + vel·age under-estimated
//  an accelerating hand and the true residual was only ~0.4–1.4 rev — a FALSE
//  trip. FW 27: (1) an exceed tick counts toward the latch only when the raw
//  anchor age is <= HAND_DEV_FRESH_US (30 ms), else it is a stale skip; (2)
//  MOTOR_FB_STALE covers the hand, so a hand blind past 150 ms stops instead.

// An armed session with the hand lane latched at zero residual: output is
// ENABLED by the fault_step, and the next interp tick is the enable edge (the
// lane clears and the staged frame of that tick re-latches it — the live
// stream-then-arm shape the existing hand-deviation test uses).
static void fw27_arm_hand_session() {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fault_set_mpc_active(true);
  fake_set_udp_last_rx_us(fake_mono_us());
  axes[0].heartbeat_seen = true;                     // leg 0 present + fresh: leg guards quiet
  stamp_hb(0);
  axes[0].pos_timestamp_us = fake_mono_us();
  axes[HAND_AXIS].heartbeat_seen = true;
  stamp_hb(HAND_AXIS);
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
  JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
  sp.flags = 0x4u;                                   // HAS_HAND, u0 = 0 = the encoder
  interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
  interp_isr();
  fault_step();
  REQUIRE(interp_output_enabled());
  REQUIRE(fault_guard_mode() != JbUdp::GuardMode::ESTOP);
}

// One 500 Hz tick with the hand commanded to `cmd` while its encoder anchor is
// `age_us` old. The encoder position is back-solved so the AGE-EXTRAPOLATED
// anchor is exactly `fb_ex` for any age — the residual (cmd − fb_ex) is then
// identical across ages and only the freshness differs.
static void fw27_hand_tick(uint16_t seq, float cmd, float fb_ex, float fbv, uint64_t age_us) {
  fake_advance(INTERP_PERIOD_US);
  stamp_hb(0);
  axes[0].pos_timestamp_us = fake_mono_us();
  stamp_hb(HAND_AXIS);
  const float fb = fb_ex - fbv * (float)((double)age_us * 1e-6);
  write_pos_vel(hand_axis(), fb, fbv, fake_mono_us() - age_us);
  JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
  sp.u0[HAND_AXIS] = cmd;
  sp.flags = 0x4u;                                   // HAS_HAND
  interp_on_setpoint(seq, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
  interp_isr();
  fake_set_udp_last_rx_us(fake_mono_us());
}

// attempt 2's snapshot (residual 9.6116 − 6.3405 = 3.27 rev) and the brief's
// 4.5 rev-ahead case — both past MAX_DEVIATION_HAND_REV (2.5).
struct Fw27Case { float cmd; float fb_ex; };
static const Fw27Case kFw27Cases[] = { {9.6116f, 6.3405f}, {9.6116f, 5.1116f} };

TEST_CASE("FW 27: an exceed on a STALE anchor (attempt 2: 99.9 ms) does NOT latch; it counts a stale skip") {
  for (const Fw27Case& c : kFw27Cases) {
    for (uint64_t age_us : {(uint64_t)HAND_DEV_FRESH_US + 1u, (uint64_t)99'900u}) {
      CAPTURE(c.cmd); CAPTURE(c.fb_ex); CAPTURE(age_us);
      fw27_arm_hand_session();
      fw27_hand_tick(2, c.cmd, c.fb_ex, 51.30f, age_us);
      // Not vacuous: the lane computed the residual and it IS past the band.
      REQUIRE(interp_hand_lane_active());
      CHECK(interp_hand_dev_last() == doctest::Approx(c.cmd - c.fb_ex).epsilon(0.001));
      CHECK(interp_hand_dev_last() > MAX_DEVIATION_HAND_REV);
      CHECK(interp_hand_dev_over_ticks() == 0u);     // the latch's counter did not move
      CHECK(interp_hand_dev_stale_skips() == 1u);    // … the stale skip did
      fault_step();
      CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);
      CHECK(fault_state() != JbUdp::FaultState::MAX_DEVIATION);
      CHECK(fault_state() != JbUdp::FaultState::MOTOR_FB_STALE);   // 99.9 ms < 150 ms
      CHECK(interp_output_enabled());
      // `hand7 reset` zeroes the new counter with its siblings.
      interp_hand_counters_reset();
      CHECK(interp_hand_dev_stale_skips() == 0u);
    }
  }
}

TEST_CASE("FW 27: the SAME residual on a FRESH anchor (5 ms, and the 30 ms bound) still latches on ONE tick") {
  for (const Fw27Case& c : kFw27Cases) {
    for (uint64_t age_us : {(uint64_t)5'000u, (uint64_t)HAND_DEV_FRESH_US}) {
      CAPTURE(c.cmd); CAPTURE(c.fb_ex); CAPTURE(age_us);
      fw27_arm_hand_session();
      fw27_hand_tick(2, c.cmd, c.fb_ex, 51.30f, age_us);
      CHECK(interp_hand_dev_over_ticks() == 1u);     // one tick is enough — unchanged
      CHECK(interp_hand_dev_stale_skips() == 0u);
      fault_step();
      CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
      CHECK(fault_state() == JbUdp::FaultState::MAX_DEVIATION);
      CHECK(fault_max_dev_leg() == HAND_AXIS);
      CHECK(fault_max_dev_value() == doctest::Approx(c.cmd - c.fb_ex).epsilon(0.001));
      CHECK(fault_max_dev_u0() == doctest::Approx(c.cmd).epsilon(0.001));
      CHECK(fault_max_dev_enc() == doctest::Approx(c.fb_ex).epsilon(0.001));
      CHECK(interp_output_enabled() == false);
    }
  }
}

TEST_CASE("FW 27: hand feedback older than MOTOR_FB_STALENESS suppresses output recoverably (axis 6 joins the legs)") {
  fw27_arm_hand_session();
  fw27_hand_tick(2, 0.0f, 0.0f, 0.0f, 0u);           // the enable-edge tick re-latches the lane
  REQUIRE(interp_hand_lane_active());
  fault_step();
  REQUIRE(interp_output_enabled());

  // The hand's encoder freezes; the legs, both heartbeats and the link stay
  // fresh, so the ONLY stale thing is axis 6. Pre FW 27 nothing noticed.
  fake_advance(200'000);                             // > 150 ms, < the 250 ms setpoint staleness
  stamp_hb(0);
  axes[0].pos_timestamp_us = fake_mono_us();
  stamp_hb(HAND_AXIS);
  fake_set_udp_last_rx_us(fake_mono_us());
  fault_step();
  CHECK(fault_state() == JbUdp::FaultState::MOTOR_FB_STALE);
  CHECK(interp_output_enabled() == false);           // suppressed — the hand TX shares this gate
  CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);   // recoverable, NOT latched

  // Hand feedback returns → output re-enables (exactly the leg contract).
  write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
  fault_step();
  CHECK(fault_state() != JbUdp::FaultState::MOTOR_FB_STALE);
  CHECK(interp_output_enabled());
}

TEST_CASE("FW 27: a stale hand does NOT suppress when the lane is idle or axis 6 was never seen") {
  SUBCASE("lane idle (a legs-only session): the hand's age is not a reason to stop the legs") {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    fault_set_mpc_active(true);
    fake_set_udp_last_rx_us(fake_mono_us());
    axes[0].heartbeat_seen = true;
    stamp_hb(0);
    axes[0].pos_timestamp_us = fake_mono_us();
    axes[HAND_AXIS].heartbeat_seen = true;
    stamp_hb(HAND_AXIS);
    write_pos_vel(hand_axis(), 0.0f, 0.0f, fake_mono_us());
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));   // no HAS_HAND
    interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
    REQUIRE(interp_have_latched());
    REQUIRE_FALSE(interp_hand_lane_active());
    fake_advance(200'000);
    stamp_hb(0);
    axes[0].pos_timestamp_us = fake_mono_us();
    stamp_hb(HAND_AXIS);
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    CHECK(fault_state() != JbUdp::FaultState::MOTOR_FB_STALE);
    CHECK(interp_output_enabled());
  }
  SUBCASE("lane active but axis 6 never reported (ts 0): the interp sends it nothing, so nothing to stop") {
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    fault_set_mpc_active(true);
    fake_set_udp_last_rx_us(fake_mono_us());
    axes[0].heartbeat_seen = true;
    stamp_hb(0);
    axes[0].pos_timestamp_us = fake_mono_us();
    REQUIRE(axes[HAND_AXIS].pos_timestamp_us == 0u);
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
    sp.flags = 0x4u;                                 // HAS_HAND
    interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    interp_isr();
    REQUIRE(interp_hand_lane_active());
    fake_advance(200'000);
    stamp_hb(0);
    axes[0].pos_timestamp_us = fake_mono_us();
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    CHECK(fault_state() != JbUdp::FaultState::MOTOR_FB_STALE);
    CHECK(interp_output_enabled());
  }
}

// ── FW 34 (BUG-20): the setpoint age runs from the arm edge on a re-arm ──────────
// s_last_setpoint_us survives across arms (interp_reset() has no runtime caller),
// so pre-FW 34 a 10 Hz fault tick landing between a re-arm and the first new
// setpoint saw the PREVIOUS stream's age and latched SETPOINT_STALE (2026-10-27
// P3 Sitting: armed .376, latched .493, first new frame .412). Recipe as Gap 5:
// only the setpoint clock is allowed to age; heartbeat / encoder / link are
// re-stamped to now before every tick.
static void jb75_keep_fresh() {
  stamp_hb(0);
  axes[0].pos_timestamp_us = fake_mono_us();
  fake_set_udp_last_rx_us(fake_mono_us());
}
static void jb75_setpoint(uint16_t seq) {
  JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
  interp_on_setpoint(seq, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
  interp_isr();
}
// A bridge that streamed once (stream A), was disarmed and has sat idle well past
// the staleness bound, then re-armed: returns with the arm edge at fake now.
static void jb75_rearm_after_old_stream() {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  axes[0].heartbeat_seen = true;
  axes[0].pos_rev = 0.0f;
  jb75_keep_fresh();
  fault_set_mpc_active(true);
  jb75_setpoint(10);
  fault_step();
  REQUIRE(fault_state() == JbUdp::FaultState::NONE);
  REQUIRE(interp_output_enabled());
  fault_set_mpc_active(false);
  fake_advance(SETPOINT_STALENESS_US + 500'000);
  jb75_keep_fresh();
  fault_step();
  REQUIRE(fault_guard_mode() == JbUdp::GuardMode::DISABLED);
  REQUIRE_FALSE(interp_output_enabled());
  fault_set_mpc_active(true);              // the re-arm edge
}

TEST_CASE("FW 34 (BUG-20): a re-arm does not latch on the previous stream's age") {
  SUBCASE("a fault tick before the first new setpoint holds output off, no latch") {
    jb75_rearm_after_old_stream();
    fake_advance(20'000);                  // inside the 16-37 ms window
    jb75_keep_fresh();
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::NONE);        // pre-FW 34: SETPOINT_STALE
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ENABLED);
    CHECK_FALSE(interp_output_enabled());  // the latched base is still stream A's

    // The new stream's first frame (seq restarts low: the dead-stream re-baseline
    // must still fire, which is why the arm does not stamp s_last_setpoint_us).
    fake_advance(10'000);
    jb75_keep_fresh();
    jb75_setpoint(0);
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::NONE);
    CHECK(interp_output_enabled());
    CHECK(interp_last_setpoint_us() == fake_mono_us());     // seq 0 was ACCEPTED
  }

  SUBCASE("a dead stream after the arm still latches one staleness bound after the ARM") {
    jb75_rearm_after_old_stream();
    fake_advance(SETPOINT_STALENESS_US - 10'000);   // just inside the grace
    jb75_keep_fresh();
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::NONE);
    CHECK_FALSE(interp_output_enabled());
    fake_advance(20'000);                            // 10 ms past the bound
    jb75_keep_fresh();
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::SETPOINT_STALE);
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    CHECK_FALSE(interp_output_enabled());
  }

  SUBCASE("a live stream that dies mid-Attempt still trips from its last frame") {
    jb75_rearm_after_old_stream();
    fake_advance(10'000);
    jb75_keep_fresh();
    jb75_setpoint(0);
    fault_step();
    REQUIRE(interp_output_enabled());
    fake_advance(SETPOINT_STALENESS_US + 1);
    jb75_keep_fresh();
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::SETPOINT_STALE);
  }
}

// =============================================================================
//  BUG-19 (FW 35) — the fault machine asks for the suppress-edge hold flush ONLY
//  for a MOTOR_FB_STALE suppression. End to end through the real interp: a
//  streaming leg whose feedback goes stale gets ONE vel_ff = 0 frame at its last
//  input_pos; a SETPOINT_STALE E-STOP from the same streaming state gets none.
// =============================================================================
namespace jb73fm {
static void arm_streaming_leg0() {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fault_set_mpc_active(true);
  axes[0].heartbeat_seen = true;
  stamp_hb(0);
  axes[0].pos_rev = 0.0f;
  axes[0].pos_timestamp_us = fake_mono_us();
  fake_set_udp_last_rx_us(fake_mono_us());
  JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
  sp.v0[0] = 1.0f;                                  // a real vel_ff on the wire (Taylor mode)
  interp_on_setpoint(0, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
  fault_step();
  REQUIRE(interp_output_enabled());
  fake_clear_sent();
  interp_isr();                                     // one streamed tick: leg0 is on the wire
  REQUIRE(fake_sent_count_cmd(ODriveCmd::set_input_pos) == 1);
}
static int16_t sent_vel(size_t i) { int16_t v; memcpy(&v, &fake_sent_at(i).buf[4], 2); return v; }
}  // namespace jb73fm

TEST_CASE("BUG-19: MOTOR_FB_STALE suppression flushes one vel_ff = 0 hold frame; a guard E-STOP flushes nothing") {
  using namespace jb73fm;
  SUBCASE("fb-stale: one hold frame per lane, then silence") {
    arm_streaming_leg0();
    uint8_t last[8]; memcpy(last, fake_sent_at(0).buf, 8);
    REQUIRE(sent_vel(0) != 0);
    fake_advance(200'000);                           // ONLY the encoder ages (> 150 ms)
    stamp_hb(0);
    fake_set_udp_last_rx_us(fake_mono_us());
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp)); sp.v0[0] = 1.0f;
    interp_on_setpoint(1, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));   // stream still live
    fault_step();
    REQUIRE(fault_state() == JbUdp::FaultState::MOTOR_FB_STALE);
    REQUIRE_FALSE(interp_output_enabled());
    fake_clear_sent();
    interp_isr();
    REQUIRE(fake_sent_count_cmd(ODriveCmd::set_input_pos) == 1);
    CHECK(memcmp(fake_sent_at(0).buf, last, 4) == 0);   // the LAST transmitted input_pos
    CHECK(sent_vel(0) == 0);
    fake_clear_sent();
    interp_isr();
    CHECK(fake_sent_count() == 0);
    CHECK(interp_hold_flush_edges() == 1u);
  }
  SUBCASE("SETPOINT_STALE E-STOP: nothing transmitted on the disable edge") {
    arm_streaming_leg0();
    fake_advance(SETPOINT_STALENESS_US + 1);         // ONLY the setpoint ages
    stamp_hb(0);
    axes[0].pos_timestamp_us = fake_mono_us();
    fake_set_udp_last_rx_us(fake_mono_us());
    fault_step();
    REQUIRE(fault_state() == JbUdp::FaultState::SETPOINT_STALE);
    fake_clear_sent();
    interp_isr();
    CHECK(fake_sent_count() == 0);
    CHECK(interp_hold_flush_edges() == 0u);
  }
}

// ── FW 48 (BUG-123): the Hand lane stall latch ───────────────────────────────
//  leg_activate.cpp's lane stall guard IDLEs the Hand and calls
//  fault_notify_hand_stall(). Here: what the latch does to the rest of the
//  machine. It must hold the Legs exactly as a Hand ODrive fault does (guard
//  ESTOP, output gated), name itself even under an older frozen guard reason
//  (2026-11-20: MAX_DEVIATION latched 80 ms into the stall), and release only on
//  the operator's CLEAR_ERRORS.
// Armed with a live leg-0 stream (the same setup as the BUG-19 cases above).
static void fw47_arm() {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  fault_set_mpc_active(true);
  axes[0].heartbeat_seen = true;
  stamp_hb(0);
  axes[0].pos_rev = 0.0f;
  axes[0].pos_timestamp_us = fake_mono_us();
  fake_set_udp_last_rx_us(fake_mono_us());
  JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
  interp_on_setpoint(0, reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
  fault_step();
  REQUIRE(interp_output_enabled());
  interp_isr();                                     // one streamed tick: the interp has latched
}
TEST_CASE("FW 48: a Hand stall latch holds the Legs, names HAND_STALL and releases only on clear-errors") {
  SUBCASE("on a live stream") {
    fw47_arm();
    fault_notify_hand_stall();
    CHECK(fault_hand_stall_latched());
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::HAND_STALL);
    CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
    CHECK_FALSE(interp_output_enabled());
    fake_clear_sent();
    interp_isr();
    CHECK(fake_sent_count() == 0);                 // nothing streams to any axis
    // Nothing but the operator releases it: a fresh setpoint and ticks do not.
    for (int k = 0; k < 20; ++k) {
      fake_advance(100000);
      stamp_hb(0);
      axes[0].pos_timestamp_us = fake_mono_us();
      fake_set_udp_last_rx_us(fake_mono_us());
      JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
      interp_on_setpoint((uint16_t)(10 + k), reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
      fault_step();
      CHECK(fault_state() == JbUdp::FaultState::HAND_STALL);
    }
    fault_notify_clear_errors();
    CHECK_FALSE(fault_hand_stall_latched());
    fault_step();
    CHECK(fault_state() != JbUdp::FaultState::HAND_STALL);
    CHECK(fault_guard_mode() != JbUdp::GuardMode::ESTOP);
  }
  SUBCASE("under an earlier MAX_DEVIATION latch the reported reason is HAND_STALL") {
    fw47_arm();
    axes[0].pos_rev = MAX_DEVIATION_REV + 5.0f;    // the base stays at 0: a leg deviation latch
    fault_step();
    REQUIRE(fault_state() == JbUdp::FaultState::MAX_DEVIATION);
    fault_notify_hand_stall();
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::HAND_STALL);
    CHECK(fault_max_dev_leg() == 0);               // the older snapshot is kept
    fault_notify_clear_errors();
    axes[0].pos_rev = 0.0f;
    fault_step();
    CHECK(fault_state() == JbUdp::FaultState::NONE);
  }
  SUBCASE("boot clears it") {
    fault_notify_hand_stall();
    fault_machine_init();
    CHECK_FALSE(fault_hand_stall_latched());
  }
}

// ── BUG-105 (FW 49) through the REAL fault machine ───────────────────────────
//  The interp-only BUG-105 cases (test_leg_interp.cpp) drop output by calling
//  interp_set_output_enabled(false) directly. Here the drop comes from a real
//  latch evaluated by fault_step(): a guard E-STOP (MOTOR_OVERSPEED, the Hand's
//  own bound) and the FW 48 HAND_STALL latch (fault_notify_hand_stall(), the
//  call leg_activate.cpp's stall guard makes; the guard's own IDLE frame is
//  leg_activate.cpp's and is not compiled into this binary). The Hand streams
//  a live vel_ff (legacy tau == 0, so no FW 22 drain); on the edge exactly ONE
//  set_input_pos reaches the wire, for the Hand, at the last transmitted
//  input_pos with vel_ff 0 and torque 0, no Leg frame, and nothing after it.
namespace bug105fm {
struct Cap { bool seen = false; uint8_t buf[8] = {0}; };
static size_t capture(Cap out[NUM_AXES]) {
  size_t n = 0;
  for (int a = 0; a < NUM_AXES; ++a) out[a] = Cap{};
  for (size_t i = 0; i < fake_sent_count(); ++i) {
    const SentFrame& f = fake_sent_at(i);
    if ((f.id & ODrive::CMD_ID_MASK) != ODriveCmd::set_input_pos) continue;
    const uint8_t a = ODrive::axis_of(f.id);
    if (a < NUM_AXES) { out[a].seen = true; memcpy(out[a].buf, f.buf, 8); ++n; }
  }
  return n;
}
static int16_t vel_of(const Cap& c) { int16_t v; memcpy(&v, &c.buf[4], 2); return v; }
static int16_t tor_of(const Cap& c) { int16_t t; memcpy(&t, &c.buf[6], 2); return t; }
// Keep every guard input fresh (leg 0, the Hand encoder AT its own command, the
// link) so the only thing that can change output is the latch under test.
static void keep_fresh(float hand_pos, float hand_vel) {
  stamp_hb(0);
  axes[0].pos_timestamp_us = fake_mono_us();
  stamp_hb(HAND_AXIS);
  write_pos_vel(hand_axis(), hand_pos, hand_vel, fake_mono_us());
  fake_set_udp_last_rx_us(fake_mono_us());
}
// Armed (fw27_arm_hand_session), then a Hand stream at u0 1.0 rev, v0 4 rev/s
// for 5 ticks with a fault_step between each, as on target. Returns the last
// Hand frame on the wire, which carries a live vel_ff.
static Cap stream_live_hand() {
  fw27_arm_hand_session();
  Cap last[NUM_AXES];
  for (uint16_t k = 0; k < 5; ++k) {
    fake_advance(INTERP_PERIOD_US);
    keep_fresh(axes[HAND_AXIS].target_pos_rev, 0.0f);
    JbUdp::SetpointPayload sp; memset(&sp, 0, sizeof(sp));
    sp.u0[HAND_AXIS] = 1.0f; sp.v0[HAND_AXIS] = 4.0f;
    sp.flags = 0x4u;                                   // HAS_HAND, Taylor (no u1)
    interp_on_setpoint((uint16_t)(2 + k), reinterpret_cast<const uint8_t*>(&sp), sizeof(sp));
    fault_step();
    REQUIRE(interp_output_enabled());
    fake_clear_sent(); interp_isr(); capture(last);
  }
  REQUIRE(last[HAND_AXIS].seen);
  REQUIRE(vel_of(last[HAND_AXIS]) != 0);               // not vacuous: a live vel_ff is on the wire
  REQUIRE(tor_of(last[HAND_AXIS]) == 0);               // legacy tau == 0: no FW 22 drain
  return last[HAND_AXIS];
}
// The edge tick and 25 more (fault_step between each): exactly one Hand hold
// frame, then silence, and the latch never lets go.
static void expect_one_hold_frame(const Cap& last, uint8_t want_state) {
  fault_step();
  REQUIRE(fault_state() == want_state);
  REQUIRE(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
  REQUIRE_FALSE(interp_output_enabled());
  Cap got[NUM_AXES];
  fake_clear_sent(); interp_isr();
  CHECK(capture(got) == 1);                            // the Hand alone
  CHECK(fake_sent_count() == 1);                       // and no other frame at all
  CHECK_FALSE(got[0].seen);                            // an E-STOP flushes no Leg
  REQUIRE(got[HAND_AXIS].seen);
  CHECK(memcmp(got[HAND_AXIS].buf, last.buf, 4) == 0); // the LAST transmitted input_pos
  CHECK(vel_of(got[HAND_AXIS]) == 0);
  CHECK(tor_of(got[HAND_AXIS]) == 0);
  CHECK(interp_hold_flush_edges() == 1u);
  for (int k = 0; k < 25; ++k) {
    fake_advance(INTERP_PERIOD_US);
    keep_fresh(axes[HAND_AXIS].pos_rev, 0.0f);
    fault_step();
    CHECK(fault_state() == want_state);
    fake_clear_sent(); interp_isr();
    CHECK(fake_sent_count() == 0);
  }
  CHECK(interp_hold_flush_edges() == 1u);
}
}  // namespace bug105fm

TEST_CASE("BUG-105 through the fault machine: a real guard E-STOP or HAND_STALL latch puts ONE Hand hold frame out (last pos, vel_ff 0), then nothing") {
  using namespace bug105fm;
  SUBCASE("guard E-STOP: the Hand's own MOTOR_OVERSPEED latch") {
    const Cap last = stream_live_hand();
    keep_fresh(axes[HAND_AXIS].pos_rev, HAND_MAX_MOTOR_VEL_RPS + 5.0f);
    expect_one_hold_frame(last, JbUdp::FaultState::MOTOR_OVERSPEED);
  }
  SUBCASE("HAND_STALL: the FW 48 latch via fault_notify_hand_stall()") {
    const Cap last = stream_live_hand();
    fault_notify_hand_stall();
    expect_one_hold_frame(last, JbUdp::FaultState::HAND_STALL);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// FW 53 U4 (I-FW-19, fw53_design.md 3.3): the deferred stow's CONFIRM phase.
// The commanded completion no longer IDLEs the Legs; the encoders must confirm
// Stow (leg_idle_permitted), else after GENTLE_MOVE_TIMEOUT_S the Legs are frozen,
// HELD, and the pending stow cleared.
// ═════════════════════════════════════════════════════════════════════════════

static constexpr uint8_t CMD_INPUT_POS = ODriveCmd::set_input_pos;
static const uint64_t CONFIRM_TIMEOUT_US = (uint64_t)(JBOp::GENTLE_MOVE_TIMEOUT_S * 1.0e6f);

// A present Leg reporting `st` in a fresh heartbeat, its encoder at (pos, vel) now.
static void stage_leg(uint8_t i, uint8_t st, float pos, float vel) {
  axes[i].heartbeat_seen = true;
  axes[i].active_errors = 0;
  axes[i].axis_state = st;
  stamp_hb(i);
  write_pos_vel(axes[i], pos, vel, fake_mono_us());
}
// The six Legs Closed-loop at Stow, except `lag` (0xFF: none) at (lag_pos, lag_vel).
static void stage_six(uint8_t lag, float lag_pos, float lag_vel) {
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    if (i == lag) stage_leg(i, ST_CL, lag_pos, lag_vel);
    else          stage_leg(i, ST_CL, 0.0f, 0.0f);
  }
}
// Loss → confirmed reconnect → one ISR tick sets the COMMANDED completion (the
// Legs read 0 rev at begin, so the profile is done in one tick). Leaves the clock
// at the reconnect tick; the next fault_step is the completion tick.
static void loss_reconnect_complete() {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  stage_six(0xFF, 0.0f, 0.0f);
  fake_advance(3 * (uint64_t)CAN_AXIS_SILENCE_TIMEOUT_US);
  fault_step();                                  // loss: armed
  REQUIRE(s_stow_pending);
  stage_six(0xFF, 0.0f, 0.0f);
  fault_step();                                  // reconnect: the stow starts
  REQUIRE(s_stowing);
  interp_isr();
  REQUIRE(interp_stow_complete());
}

TEST_CASE("FW 53: commanded completion does not IDLE until the encoders confirm Stow") {
  loss_reconnect_complete();
  // Leg 3 lags 0.5 rev behind the commanded profile, still descending.
  stage_six(3, 0.5f, -1.0f);
  const uint32_t refusals = leg_deenergise_refusals();
  fake_clear_sent();
  fault_step();                                  // completion tick: CONFIRM, refused
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == 0);
  CHECK(leg_deenergise_refusals() == refusals);  // waiting is not a refusal: no census,
  CHECK(leg_held_mask() == 0);                   // no HELD (the predicate is asked first)
  CHECK(s_stowing);
  CHECK(s_stow_pending);
  CHECK(interp_stow_active());
  CHECK(interp_output_enabled());
  // The stream keeps commanding Stow (one setpoint per present Leg per ISR tick).
  fake_clear_sent();
  interp_isr();
  CHECK(fake_sent_count_cmd(CMD_INPUT_POS) == NUM_LEGS);
  // 100 ms later Leg 3 is still 0.2 rev up: nothing.
  fake_advance(100000);
  stage_six(3, 0.2f, -1.0f);
  fake_clear_sent();
  fault_step();
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == 0);
  CHECK(s_stowing);
  // It arrives and settles: all six IDLE'd through the gate, the stow ends.
  fake_advance(100000);
  stage_six(0xFF, 0.0f, 0.0f);
  fake_clear_sent();
  fault_step();
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == NUM_LEGS);
  CHECK_FALSE(s_stowing);
  CHECK_FALSE(s_stow_pending);
  CHECK_FALSE(interp_stow_active());
  CHECK(leg_held_mask() == 0);
}

TEST_CASE("FW 53: a Leg that never confirms: after 10 s the stream ends, freeze, no IDLE, pending cleared, HELD") {
  // Leg 3 blocked 0.5 rev up, at rest; Leg 2 dropped out (IDLE) earlier.
  loss_reconnect_complete();
  const auto stage = [] { stage_six(3, 0.5f, 0.0f); stage_leg(2, ST_IDLE, 0.0f, 0.0f); };
  stage();
  fault_step();                                  // CONFIRM starts (t0 = this tick)
  REQUIRE(s_stowing);
  // Just inside the timeout: still confirming, nothing sent.
  fake_advance(CONFIRM_TIMEOUT_US - 100000);
  stage();
  fake_clear_sent();
  fault_step();
  CHECK(s_stowing);
  CHECK(fake_sent_count() == 0);
  // At the timeout: no IDLE; the stream ends; every Closed-loop Leg frozen at its
  // measured position (one set_input_pos each), HELD; the pending stow cleared.
  // The dropped (IDLE) Leg 2 is neither frozen nor HELD.
  // FW 53 audit follow-up: Leg 3 still creeps (-0.4 rev/s) at the timeout tick, and
  // its hold is the MEASURED 0.5 rev exactly, with no TRAP_TRAJ stopping distance
  // (the Legs may be in PASSTHROUGH, where that would be an unprofiled step).
  // Mutation caught: the stopping distance kept (target 0.5 - 0.16/60 rev).
  fake_advance(100000);
  stage();
  stage_leg(3, ST_CL, 0.5f, -0.4f);
  REQUIRE(leg_freeze_target(0.5f, -0.4f, STOW_WINDOW_MIN_REV, STROKE_MAX_REV[3]) != 0.5f);
  fake_clear_sent();
  fault_step();
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == 0);
  CHECK(fake_sent_count_cmd(CMD_INPUT_POS) == NUM_LEGS - 1);
  const auto hold3 = ODrive::encode_set_input_pos(3, ODrive::leg_sign(3, 0.5f), 0, 0);
  bool saw_hold3 = false;
  for (size_t k = 0; k < fake_sent_count(); ++k)
    if (fake_sent_at(k).id == hold3.id && memcmp(fake_sent_at(k).buf, hold3.buf, 8) == 0) saw_hold3 = true;
  CHECK(saw_hold3);                              // Leg 3 held where it is, not sent to 0
  CHECK(leg_held_mask() == 0x3B);
  CHECK_FALSE(s_stowing);
  CHECK_FALSE(fault_stow_pending());
  CHECK_FALSE(interp_stow_active());
  // Nothing more afterwards.
  fake_advance(100000);
  stage();
  fake_clear_sent();
  fault_step();
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == 0);
}

// FW 53 audit follow-up (the HELD race): the CONFIRM loop asks the predicate once
// (true), then each Leg's IDLE through the wrapper, which asks it again. A frame
// landing in between (staged here as Leg 4's heartbeat: fresh at the caller's `now`,
// 510 ms old at the wrapper's micros64()) refuses the IDLEs. That refusal must be
// silent: no HELD (the host's monitors fail on the first HELD), no census, and the
// IDLE is asked again next tick. Mutation caught: the CONFIRM loop back on Who::STOW
// (HELD 0x3F, census +6).
TEST_CASE("FW 53 audit: a refusal racing the CONFIRM pre-check is silent and retried next tick") {
  loss_reconnect_complete();
  stage_six(3, 0.5f, -1.0f);                     // Leg 3 lagging: CONFIRM waits
  fault_step();
  REQUIRE(s_stow_confirming);
  fake_advance(100000);
  stage_six(0xFF, 0.0f, 0.0f);                   // all six at Stow, heartbeats at T
  fake_advance(510000);                          // T + 510 ms
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {       // fresh frames, except Leg 4's heartbeat
    write_pos_vel(axes[i], 0.0f, 0.0f, fake_mono_us());
    if (i != 4) stamp_hb(i);
  }
  const uint64_t now_pre = fake_mono_us() - 100000;   // the caller's earlier `now`: Leg 4 410 ms
  REQUIRE(leg_idle_permitted(now_pre));
  REQUIRE_FALSE(leg_idle_permitted(fake_mono_us()));
  const uint32_t refusals = leg_deenergise_refusals();
  fake_clear_sent();
  stow_confirm_step(now_pre);
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == 0);
  CHECK(leg_held_mask() == 0);
  CHECK(leg_deenergise_refusals() == refusals);
  CHECK(s_stowing);
  CHECK(s_stow_confirming);
  // Next tick, Leg 4's heartbeat is in: all six IDLE'd, the stow ends.
  fake_advance(10000);
  stage_six(0xFF, 0.0f, 0.0f);
  fake_clear_sent();
  fault_step();
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == NUM_LEGS);
  CHECK_FALSE(s_stowing);
  CHECK(leg_held_mask() == 0);
}

TEST_CASE("FW 53: an already-IDLE (dropped) Leg does not block the others' IDLE") {
  loss_reconnect_complete();
  stage_six(0xFF, 0.0f, 0.0f);
  stage_leg(2, ST_IDLE, 1.2f, 0.0f);             // dropped out, its encoder anywhere
  fake_clear_sent();
  fault_step();
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == NUM_LEGS);   // the present six, Leg 2 included
  CHECK_FALSE(s_stowing);
  CHECK_FALSE(s_stow_pending);
  CHECK(leg_held_mask() == 0);
}

TEST_CASE("FW 53: an absent Leg is not sent an IDLE; the bench Leg is IDLE'd at once wherever it is") {
  // Bench: only Leg 0 on the bus, settled as a bench. It lags 1.0 rev up at the
  // commanded completion; the bench exemption IDLEs it in the completion tick.
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  stage_leg(0, ST_CL, 0.0f, 0.0f);
  rig_note_axis_frame(0, fake_mono_us());
  rig_note_bus(true, fake_mono_us());
  fake_advance(3 * (uint64_t)CAN_AXIS_SILENCE_TIMEOUT_US);
  fault_step();
  REQUIRE(s_stow_pending);
  stage_leg(0, ST_CL, 0.0f, 0.0f);
  rig_note_bus(true, fake_mono_us());            // still transmittable (anchor unchanged)
  fault_step();
  REQUIRE(s_stowing);
  interp_isr();
  REQUIRE(interp_stow_complete());
  stage_leg(0, ST_CL, 1.0f, 0.0f);
  REQUIRE(rig_is_bench(fake_mono_us()));
  fake_clear_sent();
  fault_step();
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == 1);  // Leg 0 only: nothing to absent node ids
  CHECK(fake_sent_at(0).id == ODrive::encode_set_state(0, ODriveState::IDLE).id);
  CHECK_FALSE(s_stowing);
  CHECK_FALSE(s_stow_pending);
}

TEST_CASE("FW 53: a guard E-STOP during CONFIRM does not interrupt it") {
  loss_reconnect_complete();
  stage_six(3, 0.5f, -1.0f);
  fault_step();                                  // CONFIRM, waiting on Leg 3
  REQUIRE(s_stowing);
  fault_notify_hand_stall();                     // latches guard ESTOP
  fake_advance(100000);
  stage_six(3, 0.3f, -1.0f);
  fake_clear_sent();
  fault_step();
  CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);
  CHECK(s_stowing);
  CHECK(interp_output_enabled());                // the stream still reaches the Legs
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == 0);
  fake_advance(100000);
  stage_six(0xFF, 0.0f, 0.0f);
  fault_step();
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == NUM_LEGS);   // IDLE'd at Stow
  CHECK_FALSE(s_stowing);
  CHECK(fault_guard_mode() == JbUdp::GuardMode::ESTOP);   // and the E-STOP still holds
}

TEST_CASE("FW 53: the bus going down again during CONFIRM re-arms; the restarted stow confirms") {
  loss_reconnect_complete();
  stage_six(3, 0.5f, 0.0f);
  fault_step();                                  // CONFIRM, waiting on Leg 3
  REQUIRE(s_stowing);
  // Re-drop: the stream ends, the latch re-arms, nothing is sent on the dead bus.
  fake_advance(3 * (uint64_t)CAN_AXIS_SILENCE_TIMEOUT_US);
  fake_clear_sent();
  fault_step();
  CHECK(fault_can_bus_down());
  CHECK(s_stow_pending);
  CHECK_FALSE(s_stowing);
  CHECK_FALSE(interp_stow_active());
  CHECK(fake_sent_count() == 0);
  // Reconnect well past the first CONFIRM's 10 s: the stow restarts from the
  // measured positions (Leg 3 at 0.5 rev) and descends; it is not timed out.
  fake_advance(CONFIRM_TIMEOUT_US);
  stage_six(3, 0.5f, 0.0f);
  fault_step();
  CHECK(s_stowing);
  CHECK(interp_stow_active());
  int guard = 0;
  while (!interp_stow_complete() && guard++ < 5000) interp_isr();
  REQUIRE(interp_stow_complete());
  stage_six(0xFF, 0.0f, 0.0f);
  fake_clear_sent();
  fault_step();
  CHECK(fake_sent_count_cmd(CMD_SETSTATE) == NUM_LEGS);
  CHECK_FALSE(s_stowing);
  CHECK_FALSE(s_stow_pending);
  CHECK(leg_held_mask() == 0);
}

// ═════════════════════════════════════════════════════════════════════════════
// FW 53 U3 Finding 1: a stow blocked at the reconnect tick (a HOME, ACTIVATE or
// DEACTIVATE still active) used to be lost: the reconnect branch cleared
// s_fatal_can_error in that pass, so the start was never evaluated again and the
// pending latch refused HOME / ACTIVATE / DEACTIVATE until a reboot. The start is
// now evaluated every tick while the bus is up and the stow is pending.
// ═════════════════════════════════════════════════════════════════════════════

TEST_CASE("FW 53: a stow blocked by a cold-start move at the reconnect tick starts once the move ends") {
  for (int which = 0; which < 3; ++which) {
    CAPTURE(which);
    reset_all();
    fake_set_clock(10'000'000, 10'000'000);
    stage_six(0xFF, 0.0f, 0.0f);
    fake_advance(3 * (uint64_t)CAN_AXIS_SILENCE_TIMEOUT_US);
    fault_step();                                // loss: armed
    REQUIRE(s_stow_pending);
    if (which == 0) fake_set_homing(true);
    if (which == 1) fake_set_activate(true);
    if (which == 2) fake_set_deactivate(true);
    stage_six(0xFF, 0.0f, 0.0f);
    fault_step();                                // reconnect while blocked
    CHECK_FALSE(fault_can_bus_down());
    CHECK(s_stow_pending);
    CHECK_FALSE(s_stowing);                      // never co-drive a Leg with the move
    fake_advance(100000);
    stage_six(0xFF, 0.0f, 0.0f);
    fault_step();                                // still blocked
    CHECK_FALSE(s_stowing);
    fake_set_homing(false); fake_set_activate(false); fake_set_deactivate(false);
    fake_advance(100000);
    stage_six(0xFF, 0.0f, 0.0f);
    fault_step();                                // the move ended: the stow starts
    CHECK(s_stowing);
    CHECK(interp_stow_active());
  }
}

TEST_CASE("FW 53: an unblocked stow still starts in the reconnect tick, and never while the bus is down") {
  reset_all();
  fake_set_clock(10'000'000, 10'000'000);
  stage_six(0xFF, 0.0f, 0.0f);
  fake_advance(3 * (uint64_t)CAN_AXIS_SILENCE_TIMEOUT_US);
  fault_step();                                  // loss
  CHECK_FALSE(s_stowing);
  fake_advance(100000);
  fault_step();                                  // still down: no start
  CHECK_FALSE(s_stowing);
  stage_six(0xFF, 0.0f, 0.0f);
  fault_step();                                  // reconnect tick
  CHECK(s_stowing);
}
