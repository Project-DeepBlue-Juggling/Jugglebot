// =============================================================================
//  leg_homing.cpp — Phase 9b firmware homing (velocity-limited move to hardstop)
// =============================================================================
//  Port of can_node.py `_home_motor_steps` (~:1387). The reference recipe:
//    set CLOSED_LOOP → VELOCITY/VEL_RAMP → vel/curr limits (abs(speed)*2,
//    limit+headroom) → settle → set_input_vel(speed) → poll Iq, EMA it, trip
//    when |EMA| ≥ limit → IDLE → set_absolute_position(home_ref).
//  Preserved EXACTLY: the current-headroom (push to limit+headroom, detect at
//  limit), the EMA weight, and the NO-position-back-off behaviour (the IDLE relax
//  plus the LEG_ABS_POS_REV offset is the recipe — there is no separate back-off
//  move).
//
//  Precondition: the leg ODrive must have usable velocity gains (flash defaults
//  or a prior SET_VEL_GAINS RPC) — like can_node, this routine sets only
//  state/mode/limits and inherits gains from the prior `_setup_odrives`. Bad
//  gains fail SAFE: the leg won't push, current won't rise, and the move aborts
//  to IDLE on the hard timeout.
// =============================================================================
#include "leg_homing.h"

#include <Arduino.h>          // __get_PRIMASK / __disable_irq / __set_PRIMASK
#include <cmath>              // fabsf
#include "canbridge_config.h"
#include "protocol_config.h"  // ODriveState, ODriveControlMode, ODriveInputMode
#include "hardware_config.h"  // Homing::
#include "udp_protocol.h"     // JbUdp::RpcStatus / BusHealth / FaultState
#include "odrive_protocol.h"
#include "axis_state.h"
#include "can_buses.h"
#include "fault_machine.h"    // fault_can_bus_down / fault_state
#include "time_base.h"        // micros64

namespace CanBridge {

// ── Sub-phase ladder (mirrors the _home_motor_steps generator's structure) ─────
enum class Phase : uint8_t {
  IDLE,         // nothing running
  SETUP,        // state/mode/limits issued
  DRIVE,        // settle elapsed → issue set_input_vel
  MONITOR,      // poll Iq EMA for the hardstop spike
  STOP_SETTLE,  // IDLE issued, let the motor stop before defining the reference
  SET_REF,      // set_absolute_position(home_ref)
};

// Settle windows. DRIVE settle mirrors can_node `yield 0.01` (10 ms, "let
// settings take effect"); STOP settle mirrors Homing::HOMING_STOP_SETTLE_MS
// (5 ms, the delay after stopping before set_absolute_position). Both are local
// firmware-impl timings, not Python-mirrored config.
static constexpr uint64_t SETTLE_DRIVE_US = 10000ull;   // 10 ms
static constexpr uint64_t SETTLE_STOP_US  = 5000ull;    // 5 ms
// Hard per-axis timeout (Homing::MOTOR_TIMEOUT_S = 30 s) — a missed hardstop
// NEVER drives the leg indefinitely; it aborts to IDLE.
static const uint64_t TIMEOUT_US = (uint64_t)(Homing::MOTOR_TIMEOUT_S * 1.0e6f);

// ── State (owned by the homing task; start-request handed off from the net task)
static volatile bool    s_start_req  = false;   // net task → homing task
static volatile uint8_t s_start_axis = 0xFF;

static Phase    s_phase = Phase::IDLE;
static uint8_t  s_axis  = 0xFF;
static float    s_iq_ema = 0.0f;
static uint64_t s_t_start_us = 0;   // homing start (overall timeout clock)
static uint64_t s_t_phase_us = 0;   // current sub-phase entry (settle clock)
static uint8_t  s_result[NUM_LEGS] = { HOMING_NONE };

// ── Bus / fault gate (mirror rpc.cpp jugglebot_commands_allowed + the fatal
//    conditions can_node checked: `fatal_error or fatal_can_error`). Never drive
//    a confirmed-dead bus or a faulted leg. The firmware's GuardMode::ESTOP is
//    the canonical `fatal_error` analogue — it is set on (estop || s_fatal_error),
//    so it covers ODrive-fatal AND every E-STOP class (notably MOTOR_OVERSPEED,
//    which fault_machine raises independent of mpc_active; a position/velocity
//    estop must abort homing, not just an ODrive-reported fatal). ───────────────
static bool homing_allowed() {
  const uint8_t h = can_buses_stats().jugglebot_health;
  if (h == JbUdp::BusHealth::WARN || h == JbUdp::BusHealth::BUS_OFF) return false;
  if (fault_can_bus_down()) return false;
  if (fault_guard_mode() == JbUdp::GuardMode::ESTOP) return false;
  return true;
}

// Stop the active axis and record the outcome. Called on success AND on every
// abort path — the leg is ALWAYS left in IDLE, never driving.
static void finish(uint8_t result) {
  if (s_axis < NUM_LEGS) {
    can_jugglebot_send(ODrive::encode_set_state(s_axis, ODriveState::IDLE));
    s_result[s_axis] = result;
  }
  s_phase = Phase::IDLE;
  s_axis  = 0xFF;
}

void homing_init() {
  s_start_req = false;
  s_phase = Phase::IDLE;
  s_axis  = 0xFF;
  s_iq_ema = 0.0f;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) s_result[i] = HOMING_NONE;
}

uint16_t homing_request(uint8_t axis) {
  using namespace JbUdp;
  if (axis >= NUM_LEGS) return RpcStatus::ERR_BAD_ARGS;   // leg-only (rejects hand / AXIS_ALL)
  if (!homing_allowed())  return RpcStatus::ERR_BUS_DOWN;
  // Reject if a homing is active or a start is already pending (idempotent).
  // IRQ-guarded so the (busy-check + latch) is atomic vs the homing task that
  // consumes s_start_req at the top of homing_step.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool busy = (s_phase != Phase::IDLE) || s_start_req;
  if (!busy) { s_start_axis = axis; s_start_req = true; }
  __set_PRIMASK(pm);
  return busy ? RpcStatus::ERR_REJECTED : RpcStatus::OK;
}

bool homing_active() { return s_phase != Phase::IDLE || s_start_req; }

uint8_t homing_result(uint8_t axis) {
  return (axis < NUM_LEGS) ? s_result[axis] : (uint8_t)HOMING_NONE;
}

void homing_step() {
  // ── Consume a pending start request (net task → homing task). ──
  if (s_phase == Phase::IDLE) {
    // Clear s_start_req AND advance s_phase out of IDLE atomically: otherwise a
    // HOME arriving between the clear and the phase write would see
    // (s_phase==IDLE && !s_start_req) and be wrongly accepted, queuing a
    // spurious re-home. With both inside the guard, a concurrent request always
    // sees busy (either s_start_req still set, or s_phase already past IDLE).
    const uint32_t pm = __get_PRIMASK(); __disable_irq();
    const bool    start = s_start_req;
    const uint8_t ax    = s_start_axis;
    s_start_req = false;
    if (start) s_phase = Phase::SETUP;
    __set_PRIMASK(pm);
    if (!start) return;
    s_axis       = ax;
    s_iq_ema     = 0.0f;
    s_t_start_us = micros64();
    s_t_phase_us = s_t_start_us;
    s_result[ax] = HOMING_RUNNING;
    // s_phase already set to SETUP inside the guard above.
    // fall through and run SETUP this tick
  }

  const uint8_t  a   = s_axis;
  const uint64_t now = micros64();

  // ── Aborts that apply in every active phase (checked before any drive). ──
  if (!homing_allowed())            { finish(HOMING_FAILED); return; }
  if (now - s_t_start_us > TIMEOUT_US) { finish(HOMING_FAILED); return; }

  switch (s_phase) {
    case Phase::SETUP: {
      bool ok = true;
      ok = can_jugglebot_send(ODrive::encode_set_state(a, ODriveState::CLOSED_LOOP)) && ok;
      ok = can_jugglebot_send(ODrive::encode_set_controller_mode(
               a, ODriveControlMode::VELOCITY, ODriveInputMode::VEL_RAMP)) && ok;
      // vel limit = |speed|*2 (headroom so travel isn't velocity-clamped),
      // curr limit = detect-limit + headroom (so the motor can push past the
      // detection threshold against the stop). Exactly can_node's args.
      ok = can_jugglebot_send(ODrive::encode_set_vel_curr_limits(
               a, fabsf(Homing::LEG_SPEED_RPS) * 2.0f,
               Homing::LEG_CURRENT_LIMIT_A + Homing::LEG_CURRENT_HEADROOM_A)) && ok;
      if (!ok) { finish(HOMING_FAILED); return; }
      s_t_phase_us = now;
      s_phase = Phase::DRIVE;
      break;
    }
    case Phase::DRIVE: {
      if (now - s_t_phase_us < SETTLE_DRIVE_US) break;   // let settings take effect
      if (!can_jugglebot_send(ODrive::encode_set_input_vel(a, Homing::LEG_SPEED_RPS, 0.0f))) {
        finish(HOMING_FAILED); return;
      }
      s_iq_ema = 0.0f;
      s_phase = Phase::MONITOR;
      break;
    }
    case Phase::MONITOR: {
      // Iq EMA (single-word atomic read). |EMA| ≥ limit ⇒ stalled on the
      // hardstop. EMA weight + threshold are can_node's exact values.
      const float iq = axes[a].iq_measured;
      s_iq_ema = Homing::EMA_WEIGHT * s_iq_ema + (1.0f - Homing::EMA_WEIGHT) * iq;
      if (fabsf(s_iq_ema) >= Homing::LEG_CURRENT_LIMIT_A) {
        can_jugglebot_send(ODrive::encode_set_state(a, ODriveState::IDLE));  // stop pushing
        s_t_phase_us = now;
        s_phase = Phase::STOP_SETTLE;
      }
      break;
    }
    case Phase::STOP_SETTLE: {
      if (now - s_t_phase_us < SETTLE_STOP_US) break;
      s_phase = Phase::SET_REF;
      break;
    }
    case Phase::SET_REF: {
      // Define the hardstop as LEG_ABS_POS_REV — the post-homing reference.
      can_jugglebot_send(ODrive::encode_set_absolute_position(a, Homing::LEG_ABS_POS_REV));
      s_result[a] = HOMING_OK;
      s_phase = Phase::IDLE;
      s_axis  = 0xFF;
      break;
    }
    default:
      finish(HOMING_FAILED);
      break;
  }
}

}  // namespace CanBridge
