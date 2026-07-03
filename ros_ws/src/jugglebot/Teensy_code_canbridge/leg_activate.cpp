// =============================================================================
//  leg_activate.cpp — Phase 11 U5 firmware activate (TRAP_TRAJ move to active pose)
// =============================================================================
//  See leg_activate.h for the design rationale. The ladder, per targeted leg:
//    SETUP   : refuse if any target leg has active_errors (legacy _gentle_move
//              parity) → seed input_pos(actual current, NON-clipped) → traj
//              vel/acc limits → POSITION/TRAP_TRAJ mode → CLOSED_LOOP
//    settle  : let CLOSED_LOOP engage + mode/limits take effect
//    COMMAND : set_input_pos(ACTIVATE_POSITION_REVS) → ODrive TRAP_TRAJ starts
//    MONITOR : stay active until every target leg reaches the active pose +
//              settles (|pos−target| ≤ tol AND |vel| ≤ tol) → mark OK, go idle.
//  The ODrive owns the trajectory; the Jetson observes completion independently
//  via telemetry. Staying active through MONITOR is what makes HOME/ACTIVATE
//  mutually exclusive for the WHOLE physical move (not just the ~10 ms fire) —
//  the firmware, not host serialization, is the safety boundary.
// =============================================================================
#include "leg_activate.h"

#include <Arduino.h>          // __get_PRIMASK / __disable_irq / __set_PRIMASK
#include <cmath>              // fabsf
#include "canbridge_config.h"
#include "protocol_config.h"  // ODriveState, ODriveControlMode, ODriveInputMode
#include "hardware_config.h"  // JBOp::, ODriveDefaults::
#include "udp_protocol.h"     // JbUdp::RpcStatus / BusHealth / GuardMode / AXIS_ALL
#include "odrive_protocol.h"
#include "axis_state.h"
#include "can_buses.h"
#include "fault_machine.h"    // fault_can_bus_down / fault_guard_mode
#include "leg_homing.h"       // homing_active (no concurrent cold-start moves)
#include "leg_deactivate.h"   // deactivate_active (no concurrent cold-start moves)
#include "time_base.h"        // micros64

namespace CanBridge {

// ── Sub-phase ladder ──────────────────────────────────────────────────────────
enum class APhase : uint8_t {
  IDLE,      // nothing running
  SETUP,     // error gate + seed + traj limits + TRAP_TRAJ mode + CLOSED_LOOP issued
  COMMAND,   // settle elapsed → command the active-pose target
  MONITOR,   // target commanded → wait for the leg(s) to reach + settle
};

// Settle window: let CLOSED_LOOP engage and the mode/limit writes take effect
// before commanding the trajectory target (mirrors leg_homing's DRIVE settle).
static constexpr uint64_t SETTLE_SETUP_US = 10000ull;   // 10 ms
// Hard timeout on the WHOLE op (SETUP → COMMAND → physical move → settle). The
// ODrive runs the trajectory autonomously; MONITOR keeps the op "active" until it
// completes, so HOME is rejected for the whole move (not just the ~10 ms fire).
static const uint64_t A_TIMEOUT_US = (uint64_t)(JBOp::GENTLE_MOVE_TIMEOUT_S * 1.0e6f);

// ── State (owned by the activate task; start-request handed off from net task) ─
static volatile bool    s_start_req  = false;   // net task → activate task
static volatile uint8_t s_start_axis = 0xFF;

static APhase   s_phase   = APhase::IDLE;
static uint8_t  s_targets = 0;        // bitmask of legs being activated (bit i = leg i)
static uint8_t  s_setup_idx = 0;      // SETUP cursor — configure ONE leg per tick
static uint64_t s_t_start_us = 0;     // ladder start (overall timeout clock)
static uint64_t s_t_phase_us = 0;     // current sub-phase entry (settle clock)
static uint8_t  s_result[NUM_LEGS] = { ACTIVATE_NONE };

// ── Bus / fault gate (mirror leg_homing::homing_allowed). Never drive a
//    confirmed-dead bus or an E-STOP'd system. ───────────────────────────────
static bool activate_allowed() {
  const uint8_t h = can_buses_stats().jugglebot_health;
  if (h == JbUdp::BusHealth::WARN || h == JbUdp::BusHealth::BUS_OFF) return false;
  if (fault_can_bus_down()) return false;
  if (fault_guard_mode() == JbUdp::GuardMode::ESTOP) return false;
  return true;
}

// Build the target-leg bitmask: AXIS_ALL → every present leg; a single leg →
// that leg iff present. Returns 0 if there is no valid target.
static uint8_t resolve_targets(uint8_t axis) {
  uint8_t mask = 0;
  if (axis == JbUdp::RpcArgs::AXIS_ALL) {
    for (uint8_t i = 0; i < NUM_LEGS; ++i) if (leg_present(i)) mask |= (uint8_t)(1u << i);
  } else if (axis < NUM_LEGS && leg_present(axis)) {
    mask = (uint8_t)(1u << axis);
  }
  return mask;
}

// Stop every targeted leg and record the outcome. Called on every abort path —
// the legs are ALWAYS left in IDLE, never driving.
static void abort_all(uint8_t result) {
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    if (!(s_targets & (uint8_t)(1u << i))) continue;
    can_jugglebot_send(ODrive::encode_set_state(i, ODriveState::IDLE));
    s_result[i] = result;
  }
  s_phase = APhase::IDLE;
  s_targets = 0;
}

void activate_init() {
  s_start_req = false;
  s_phase = APhase::IDLE;
  s_targets = 0;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) s_result[i] = ACTIVATE_NONE;
}

uint16_t activate_request(uint8_t axis) {
  using namespace JbUdp;
  if (!activate_allowed()) return RpcStatus::ERR_BUS_DOWN;
  if (homing_active() || deactivate_active()) return RpcStatus::ERR_REJECTED;  // no concurrent cold-start moves
  if (fault_stow_pending()) return RpcStatus::ERR_REJECTED;   // not during a deferred stow (adversarial-review fix: else the stow's virtual-complete IDLEs the legs)
  // MPC-stream interlock (Flash-A item 1b): reject while the MPC is actively driving
  // the legs (guard ENABLED on the Jetson) — a cold-start move must not co-drive the
  // same ODrives the 500 Hz stream commands (the interp ISR reciprocally suppresses
  // its TX during this move, item 1a).
  if (fault_mpc_active()) return RpcStatus::ERR_REJECTED;
  if (resolve_targets(axis) == 0) return RpcStatus::ERR_BAD_ARGS;  // no present target leg
  // Reject if an activate is active or a start is already pending (idempotent).
  // IRQ-guarded so (busy-check + latch) is atomic vs the activate task that
  // consumes s_start_req at the top of activate_step.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool busy = (s_phase != APhase::IDLE) || s_start_req;
  if (!busy) { s_start_axis = axis; s_start_req = true; }
  __set_PRIMASK(pm);
  return busy ? RpcStatus::ERR_REJECTED : RpcStatus::OK;
}

bool activate_active() { return s_phase != APhase::IDLE || s_start_req; }

uint8_t activate_result(uint8_t axis) {
  return (axis < NUM_LEGS) ? s_result[axis] : (uint8_t)ACTIVATE_NONE;
}

void activate_step() {
  // ── Consume a pending start request (net task → activate task). ──
  if (s_phase == APhase::IDLE) {
    const uint32_t pm = __get_PRIMASK(); __disable_irq();
    const bool    start = s_start_req;
    const uint8_t ax    = s_start_axis;
    s_start_req = false;
    if (start) s_phase = APhase::SETUP;
    __set_PRIMASK(pm);
    if (!start) return;
    s_targets = resolve_targets(ax);
    if (s_targets == 0) { s_phase = APhase::IDLE; return; }  // legs vanished between request and consume
    for (uint8_t i = 0; i < NUM_LEGS; ++i)
      if (s_targets & (uint8_t)(1u << i)) s_result[i] = ACTIVATE_RUNNING;
    s_setup_idx = 0;   // SETUP configures one leg per tick starting here
    s_t_start_us = micros64();
    s_t_phase_us = s_t_start_us;
    // s_phase already set to SETUP inside the guard; fall through and run it.
  }

  const uint64_t now = micros64();

  // ── Aborts that apply in every active phase (checked before any drive). ──
  if (!activate_allowed())               { abort_all(ACTIVATE_FAILED); return; }
  if (now - s_t_start_us > A_TIMEOUT_US)  { abort_all(ACTIVATE_FAILED); return; }

  switch (s_phase) {
    case APhase::SETUP: {
      // Configure ONE leg per tick (s_setup_idx cursor). NEVER burst all legs'
      // config frames into a single tick: 5 frames × 6 legs = 30 overflowed the
      // 16-deep CAN3 TX buffer and most legs never entered CLOSED_LOOP (only the
      // last did) — 2026-06-26. One leg/tick mirrors the proven sequential homing
      // pattern (~few frames/tick). The COMMAND phase still fires all targets in
      // one tick AFTER every leg is configured + holding, so the physical move is
      // parallel (even platform rise).
      while (s_setup_idx < NUM_LEGS && !(s_targets & (uint8_t)(1u << s_setup_idx)))
        ++s_setup_idx;
      if (s_setup_idx >= NUM_LEGS) {
        // Every target leg configured + in CLOSED_LOOP/TRAP_TRAJ holding its seed.
        s_t_phase_us = now;          // start the settle clock for COMMAND
        s_phase = APhase::COMMAND;
        break;
      }
      const uint8_t i = s_setup_idx;
      // Synchronous per-leg error gate (legacy can_node._gentle_move_steps parity:
      // "refuse to enter CLOSED_LOOP if errors present"). ANY errored target fails
      // the whole op (a coupled platform must not activate a subset — that tilts it).
      if (axes[i].active_errors != 0) { abort_all(ACTIVATE_FAILED); return; }
      // Seed the ACTUAL current position (NON-clipped) so CLOSED_LOOP holds in
      // place even at the sub-zero hardstop — the whole move is then TRAP_TRAJ-
      // profiled with no clip-snap off the hardstop. leg_sign applies the ODrive
      // convention (Jugglebot positive=extension → ODrive negative).
      const float cur = axes[i].pos_rev;  // single-word atomic read, Jugglebot conv
      bool ok = true;
      ok = can_jugglebot_send(
               ODrive::encode_set_input_pos(i, ODrive::leg_sign(i, cur), 0, 0)) && ok;
      ok = can_jugglebot_send(
               ODrive::encode_set_traj_vel_limit(i, JBOp::GENTLE_MOVE_VEL_LIMIT_RPS)) && ok;
      ok = can_jugglebot_send(
               ODrive::encode_set_traj_acc_limits(
                   i, ODriveDefaults::TRAP_ACC_LIMIT_RPS2,
                   ODriveDefaults::TRAP_DEC_LIMIT_RPS2)) && ok;
      ok = can_jugglebot_send(
               ODrive::encode_set_controller_mode(
                   i, ODriveControlMode::POSITION, ODriveInputMode::TRAP_TRAJ)) && ok;
      ok = can_jugglebot_send(
               ODrive::encode_set_state(i, ODriveState::CLOSED_LOOP)) && ok;
      if (!ok) { abort_all(ACTIVATE_FAILED); return; }
      ++s_setup_idx;   // next target leg on the next tick
      break;
    }
    case APhase::COMMAND: {
      if (now - s_t_phase_us < SETTLE_SETUP_US) break;  // let CLOSED_LOOP + mode/limits settle
      for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        if (!(s_targets & (uint8_t)(1u << i))) continue;
        // Position-only target (vel_ff = tor_ff = 0). encode_leg_setpoint clips to
        // [0,max] (defence against a bad target) + applies leg_sign. The ODrive
        // TRAP_TRAJ planner trajectories from the seeded current pos to here.
        can_jugglebot_send(
            ODrive::encode_leg_setpoint(i, JBOp::ACTIVATE_POSITION_REVS[i], 0.0f, 0.0f));
      }
      s_phase = APhase::MONITOR;  // stay active through the physical move
      break;
    }
    case APhase::MONITOR: {
      // Wait until every target leg reaches the active pose AND settles. The
      // start position is far from the target, so the position check prevents a
      // false "done" from the leg merely sitting still at the start. The leg ramps
      // up far from target (|pos−target| large), so it cannot satisfy both until
      // the move has actually completed. Timeout (above) bounds a stall.
      bool all_done = true;
      for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        if (!(s_targets & (uint8_t)(1u << i))) continue;
        const float pos = axes[i].pos_rev;  // Jugglebot conv (== ACTIVATE_POSITION_REVS conv)
        const float vel = axes[i].vel_rps;
        if (fabsf(pos - JBOp::ACTIVATE_POSITION_REVS[i]) > JBOp::TARGET_REACHED_POS_TOL_REV ||
            fabsf(vel) > JBOp::TARGET_REACHED_VEL_TOL_RPS) {
          all_done = false;
        }
      }
      if (all_done) {
        for (uint8_t i = 0; i < NUM_LEGS; ++i)
          if (s_targets & (uint8_t)(1u << i)) s_result[i] = ACTIVATE_OK;
        s_phase = APhase::IDLE;
        s_targets = 0;
      }
      break;
    }
    default:
      abort_all(ACTIVATE_FAILED);
      break;
  }
}

}  // namespace CanBridge
