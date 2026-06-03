#pragma once
// =============================================================================
//  axis_state.h — per-axis motor state cache
// =============================================================================
//  The Teensy mirror of the Jetson's MotorStateTracker (motor_state.py).
//  One AxisState per ODrive axis: 6 legs + the hand. Populated by the CAN2 RX
//  task (telemetry/heartbeat/error decode), read by the interpolator, fault
//  state machine, and telemetry uplink.
//
//  Concurrency: single-word volatile reads/writes are atomic on Cortex-M7
//  (plan §"Per-axis state cache"). The hot pos/vel/timestamp triple is read as
//  a consistent snapshot via a seqlock-style retry loop (snapshot_pos_vel),
//  mirroring the platform Teensy's getHandPosVel() in Teensy_code.ino.
//
//  Convention: pos_rev / vel_rps are in the JUGGLEBOT convention
//  (positive = leg extension), i.e. ALREADY sign-flipped from the ODrive
//  convention by the CAN RX decode (see odrive_protocol leg_sign()).
// =============================================================================

#include <cstdint>
#include "canbridge_config.h"

namespace CanBridge {

struct AxisState {
  // ── Hot — updated by CAN2 RX task ──────────────────────────────────────────
  volatile float    pos_rev          = 0.0f;   // Jugglebot convention (pos = extension)
  volatile float    vel_rps          = 0.0f;
  volatile uint64_t pos_timestamp_us = 0;      // wall-clock at last encoder update
  volatile float    iq_setpoint      = 0.0f;   // A
  volatile float    iq_measured      = 0.0f;   // A
  volatile float    temp_fet         = 0.0f;   // degC
  volatile float    temp_motor       = 0.0f;   // degC
  volatile float    bus_voltage      = 0.0f;   // V
  volatile float    bus_current      = 0.0f;   // A

  // ── Heartbeat / state — updated by CAN2 RX + fault task ────────────────────
  volatile uint32_t active_errors    = 0;      // bitmask (odrive.py ERROR_CODES)
  volatile uint32_t disarm_reason    = 0;      // bitmask
  volatile uint8_t  axis_state       = 0;      // ODrive current_state (IDLE=1, CLOSED_LOOP=8)
  volatile uint8_t  procedure_result = 0;
  volatile uint8_t  controller_mode  = 0;      // ODriveControlMode
  volatile uint8_t  input_mode       = 0;      // ODriveInputMode
  volatile bool     trajectory_done  = false;
  volatile bool     heartbeat_stale  = false;
  volatile bool     heartbeat_seen   = false;  // ever received a heartbeat
  volatile uint64_t last_heartbeat_us = 0;     // wall-clock at last heartbeat

  // ── Targets — updated by interp task ───────────────────────────────────────
  volatile float    target_pos_rev   = 0.0f;
  volatile float    target_vel_rps   = 0.0f;
  volatile float    target_torque_Nm = 0.0f;

  // ── Configured limits / gains (set via RPC) ────────────────────────────────
  float vel_limit_rps = ODriveDefaults::LEG_VEL_LIMIT_RPS;
  float curr_limit_A  = ODriveDefaults::LEG_CURR_LIMIT_A;
  float pos_gain      = 0.0f;
  float vel_p_gain    = 0.0f;
  float vel_i_gain    = 0.0f;

  // ── Seqlock for torn-read-free snapshot of the hot pos/vel/timestamp ───────
  // Bumped odd before a write, even after. Readers retry while odd or changed.
  volatile uint32_t seq = 0;
};

// 6 legs (index = node id 0..5) + hand at index HAND_AXIS (6). Defined in axis_state.cpp.
extern AxisState axes[NUM_AXES];

inline AxisState& leg(uint8_t i)  { return axes[i]; }
inline AxisState& hand_axis()     { return axes[HAND_AXIS]; }

// Writer: update the hot triple atomically (call from CAN RX on encoder frame).
inline void write_pos_vel(AxisState& a, float pos, float vel, uint64_t t_us) {
  a.seq = a.seq + 1;          // odd → write in progress
  asm volatile("" ::: "memory");
  a.pos_rev = pos;
  a.vel_rps = vel;
  a.pos_timestamp_us = t_us;
  asm volatile("" ::: "memory");
  a.seq = a.seq + 1;          // even → done
}

// Reader: consistent snapshot of the hot triple.
inline void snapshot_pos_vel(const AxisState& a, float& pos, float& vel, uint64_t& t_us) {
  uint32_t s0, s1;
  do {
    s0 = a.seq;
    pos = a.pos_rev;
    vel = a.vel_rps;
    t_us = a.pos_timestamp_us;
    asm volatile("" ::: "memory");
    s1 = a.seq;
  } while ((s0 & 1) || (s0 != s1));   // retry if writer was mid-update
}

}  // namespace CanBridge
