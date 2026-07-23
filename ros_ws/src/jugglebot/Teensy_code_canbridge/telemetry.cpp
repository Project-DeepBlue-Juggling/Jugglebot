// =============================================================================
//  telemetry.cpp — motor-state uplink + on-change diagnostics
// =============================================================================
#include "telemetry.h"

#include <cstring>
#include <cmath>
#include "canbridge_config.h"
#include "udp_protocol.h"
#include "udp_link.h"
#include "axis_state.h"
#include "can_buses.h"
#include "time_base.h"
#include "leg_homing.h"   // homing_result() — uplinked in the Diagnostic (see logbook 2026-07-05-canhub-hardening-18a-homing-result-uplink)

namespace CanBridge {

// ── On-change thresholds (only emit a Diagnostic when one is exceeded) ────────
static constexpr float DIAG_IQ_THRESH_A     = 0.5f;
static constexpr float DIAG_TEMP_THRESH_C    = 1.0f;
static constexpr float DIAG_VOLT_THRESH_V    = 0.5f;
static constexpr uint32_t DIAG_FORCE_PERIOD_US = 1000000u;   // 1 Hz per-axis refresh

// Last-published diagnostic snapshot, per axis, for change detection.
struct DiagBaseline {
  uint32_t active_errors, disarm_reason;
  uint8_t  axis_state, ctrl_mode, input_mode, flags, homing_result;
  float    iq_setpoint, temp_fet, temp_motor, bus_voltage;
  uint64_t last_sent_us;
  bool     ever_sent;
};
static DiagBaseline s_base[NUM_AXES];

void telemetry_init() {
  for (uint8_t i = 0; i < NUM_AXES; ++i) s_base[i] = DiagBaseline{};
}

// ── 100 Hz motor-state frame ──────────────────────────────────────────────────
static void send_telemetry() {
  JbUdp::TelemetryPayload t{};
  t.t_teensy_us = now_wall_us();   // wire-bound absolute timestamp — wall by contract
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    float pos, vel; uint64_t ts;
    snapshot_pos_vel(axes[i], pos, vel, ts);
    t.pos_rev[i] = pos;
    t.vel_rps[i] = vel;
  }
  udp_send_stream(JbUdp::MsgType::TELEMETRY, (const uint8_t*)&t, sizeof(t));
}

// ── Per-axis diagnostic (on-change or 1 Hz) ───────────────────────────────────
static bool diag_changed(const AxisState& a, const DiagBaseline& b) {
  if (!b.ever_sent) return true;
  if (a.active_errors != b.active_errors) return true;
  if (a.disarm_reason != b.disarm_reason) return true;
  if (a.axis_state != b.axis_state) return true;
  if (a.controller_mode != b.ctrl_mode) return true;
  if (a.input_mode != b.input_mode) return true;
  if (fabsf(a.iq_setpoint - b.iq_setpoint) > DIAG_IQ_THRESH_A) return true;
  if (fabsf(a.temp_fet   - b.temp_fet)   > DIAG_TEMP_THRESH_C) return true;
  if (fabsf(a.temp_motor - b.temp_motor) > DIAG_TEMP_THRESH_C) return true;
  if (fabsf(a.bus_voltage - b.bus_voltage) > DIAG_VOLT_THRESH_V) return true;
  return false;
}

static void send_diag(uint8_t axis) {
  AxisState& a = axes[axis];
  const uint64_t now = micros64();   // interval clock: staleness + last_sent_us age
  const bool stale = a.heartbeat_seen &&
                     (now - atomic_read_u64(&a.last_heartbeat_us) > CAN_HEARTBEAT_TIMEOUT_US);  // atomic 64-bit

  JbUdp::DiagnosticPayload d{};
  d.axis_id       = axis;
  d.axis_state    = a.axis_state;
  d.ctrl_mode     = a.controller_mode;
  d.input_mode    = a.input_mode;
  d.flags         = (stale ? 0x1u : 0x0u) | (a.heartbeat_seen ? 0x2u : 0x0u);
  d.active_errors = a.active_errors;
  d.disarm_reason = a.disarm_reason;
  d.iq_setpoint   = a.iq_setpoint;
  d.iq_measured   = a.iq_measured;
  d.temp_fet      = a.temp_fet;
  d.temp_motor    = a.temp_motor;
  d.bus_voltage   = a.bus_voltage;
  // bus_current rides along like iq_measured: no change-detect term of its own —
  // iq_setpoint (>0.5 A) is its load proxy, the 1 Hz forced refresh covers drift.
  d.bus_current   = a.bus_current;
  d.homing_result = homing_result(axis);   // real firmware outcome, per axis
  udp_send_stream(JbUdp::MsgType::DIAGNOSTIC, (const uint8_t*)&d, sizeof(d));

  DiagBaseline& b = s_base[axis];
  b.active_errors = a.active_errors; b.disarm_reason = a.disarm_reason;
  b.axis_state = a.axis_state; b.ctrl_mode = a.controller_mode; b.input_mode = a.input_mode;
  b.iq_setpoint = a.iq_setpoint; b.temp_fet = a.temp_fet;
  b.temp_motor = a.temp_motor; b.bus_voltage = a.bus_voltage;
  b.flags = d.flags;
  b.homing_result = d.homing_result;   // track for the change-detect below
  b.last_sent_us = now;
  b.ever_sent = true;
}

// ── Ball Butler ODrive diagnostic (CAN1 nodes 7/8) ───────────────────────────
// Reuses the platform DIAGNOSTIC frame with axis_id 7/8 (no protocol change); the
// Jetson stashes these under those ids and republishes on /bb/odrive_diag. Fixed
// ~1 Hz per axis (temps/currents drift slowly) — no on-change baseline needed.
static void send_bb_diag(uint8_t idx) {
  const AxisState& a = bb_axes[idx];
  const uint64_t now = micros64();   // interval clock: staleness + last_sent_us age
  const bool stale = a.heartbeat_seen &&
                     (now - atomic_read_u64(&a.last_heartbeat_us) > CAN_HEARTBEAT_TIMEOUT_US);  // atomic 64-bit
  JbUdp::DiagnosticPayload d{};
  d.axis_id       = (uint8_t)(BB_FIRST_NODE + idx);   // 7 = bb_pitch, 8 = bb_hand
  d.axis_state    = a.axis_state;
  d.ctrl_mode     = a.controller_mode;
  d.input_mode    = a.input_mode;
  // heartbeat_seen (bit1) is the host's phantom-axis gate: robot_state appends
  // BB entries only for axes whose ODrive has actually heartbeated this boot,
  // so a dark/absent BB never surfaces as an all-zero "live" motor.
  d.flags         = (stale ? 0x1u : 0x0u) | (a.heartbeat_seen ? 0x2u : 0x0u);
  d.active_errors = a.active_errors;
  d.disarm_reason = a.disarm_reason;
  d.iq_setpoint   = a.iq_setpoint;
  d.iq_measured   = a.iq_measured;
  d.temp_fet      = a.temp_fet;
  d.temp_motor    = a.temp_motor;
  d.bus_voltage   = a.bus_voltage;
  d.bus_current   = a.bus_current;
  udp_send_stream(JbUdp::MsgType::DIAGNOSTIC, (const uint8_t*)&d, sizeof(d));
}

// ── Ball Butler high-rate pos/vel estimates (CAN1 nodes 7=pitch, 8=hand) ──────
// Snapshots the bb_axes pos/vel cache (updated by the CAN1 RX decode at the ODrive
// get_encoder_estimate broadcast rate — set to 1 kHz for during-throw diagnostics)
// and emits one BB_AXIS_ESTIMATES frame. Called from telemetry_step() at
// TELEM_RATE_HZ (100 Hz) — same proven uplink context as send_telemetry(), so this
// adds no new task/concurrency surface. The 100 Hz forward downsamples the 1 kHz
// cache; the analysis fits the hand velocity trapezoid (accel/decel ramps → peak)
// and the pitch is quasi-static during the throw, so 100 Hz resolves both. Bump to
// a dedicated higher-rate task only if a sharp servo-overshoot transient must be
// caught (the ODrive side already supports it).
static void send_bb_estimates() {
  JbUdp::BbAxisEstimatesPayload e{};
  float pos, vel; uint64_t ts;
  snapshot_pos_vel(bb_axes[0], pos, vel, ts);   // node 7 = pitch
  e.pitch_pos_rev = pos; e.pitch_vel_rps = vel;
  snapshot_pos_vel(bb_axes[1], pos, vel, ts);   // node 8 = hand
  e.hand_pos_rev = pos;  e.hand_vel_rps = vel;
  e.t_bridge_us = now_wall_us();   // wire-bound absolute timestamp — wall by contract
  udp_send_stream(JbUdp::MsgType::BB_AXIS_ESTIMATES, (const uint8_t*)&e, sizeof(e));
}

// ── Commanded leg interp output (the float32 ladder result) ──────────────────
// Snapshots axes[i].target_pos_rev/target_vel_rps — what leg_interp.cpp's 500 Hz
// cubic-Hermite ladder (after the lead + stroke clamps) commands to the leg
// ODrives — at the telemetry rate. These are written every interp tick for ALL
// legs regardless of the output gate (leg_interp.cpp), so this reflects the
// float32 interpolator output even when CAN3 TX is suppressed. Used by the
// bench validation to measure the on-Teensy float32 interp residual vs the
// float64 reference directly rather than
// inferring it from the encoder. Same proven uplink context as send_telemetry().
static void send_leg_cmd() {
  JbUdp::LegCmdPayload c{};
  c.t_teensy_us = now_wall_us();   // wire-bound absolute timestamp — wall by contract
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    c.cmd_pos_rev[i] = axes[i].target_pos_rev;   // single-word atomic read
    c.cmd_vel_rps[i] = axes[i].target_vel_rps;
  }
  udp_send_stream(JbUdp::MsgType::LEG_CMD, (const uint8_t*)&c, sizeof(c));
}

// ── Cone uplink: drain CAN2 frames into CONE_FRAME UDP messages ──────────────
// Per-tick budget caps the UDP cost if CAN2 ever babbles; legitimate cone
// traffic is ~10-11 fps (10 Hz heartbeat + per-impact catch events) against the
// 100 Hz tick, so the ring (CONE_RING_CAP=16) drains in one frame per tick in
// steady state and a full ring clears in 4 ticks. Excess beyond ring+budget is
// dropped at the ring (counted — can_cone_fwd_drops, [canhealth] serial line).
static constexpr uint8_t CONE_FWD_BUDGET = 4;

void cone_uplink_step() {
  ConeFrameRec r;
  for (uint8_t i = 0; i < CONE_FWD_BUDGET && can_cone_pop(r); ++i) {
    JbUdp::ConeFramePayload p{};
    p.t_bridge_us = r.t_bridge_us;
    p.can_id = r.can_id;
    p.dlc = r.dlc;
    memcpy(p.data, r.buf, 8);
    udp_send_stream(JbUdp::MsgType::CONE_FRAME, (const uint8_t*)&p, sizeof(p));
  }
}

// ── BB command-result uplink: drain CAN1 CMD_RESULT frames into CMD_RESULT UDP ─
// Loud command-outcome channel. CMD_RESULT is low-rate (one per operator command), so the
// per-tick budget is small; a verdict never waits more than a tick to reach the
// host. Mirrors cone_uplink_step (the [canhealth] line carries the drop counter).
static constexpr uint8_t CMD_RESULT_FWD_BUDGET = 4;

void cmd_result_uplink_step() {
  CmdResultFrameRec r;
  for (uint8_t i = 0; i < CMD_RESULT_FWD_BUDGET && can_cmd_result_pop(r); ++i) {
    JbUdp::CmdResultFramePayload p{};
    p.t_bridge_us = r.t_bridge_us;
    p.can_id = r.can_id;
    p.dlc = r.dlc;
    memcpy(p.data, r.buf, 8);
    udp_send_stream(JbUdp::MsgType::CMD_RESULT, (const uint8_t*)&p, sizeof(p));
  }
}

// ── Platform-Teensy relay uplink: drain CAN3 reply frames → PLATFORM_FRAME ────
// Relay seam. Relay replies are low-rate (one per operator relay read),
// so the per-tick budget is small; a reply never waits more than a tick to reach
// the host (which correlates it to its pending read). Mirrors cone_uplink_step.
static constexpr uint8_t PLATFORM_FWD_BUDGET = 4;

void platform_uplink_step() {
  PlatformFrameRec r;
  for (uint8_t i = 0; i < PLATFORM_FWD_BUDGET && can_platform_pop(r); ++i) {
    JbUdp::PlatformFramePayload p{};
    p.t_bridge_us = r.t_bridge_us;
    p.can_id = r.can_id;
    p.dlc = r.dlc;
    memcpy(p.data, r.buf, 8);
    udp_send_stream(JbUdp::MsgType::PLATFORM_FRAME, (const uint8_t*)&p, sizeof(p));
  }
}

// ── Hand command-echo uplink ──────────────────────────────────────────────────
// Emit the latest sniffed hand Set_Input_Pos as a HAND_CMD_ECHO whenever a fresh
// one is pending (event-driven — silent while the hand is idle). The single-slot
// stash coalesces to the newest command, so at most one frame per telemetry tick;
// the host keeps the last value to fill hand_telemetry's command fields. Mirrors
// platform_uplink_step's emit.
void hand_cmd_echo_uplink_step() {
  HandCmdEchoRec r;
  if (can_hand_cmd_echo_pop(r)) {
    JbUdp::HandCmdEchoPayload p{};
    p.t_bridge_us = r.t_bridge_us;
    memcpy(p.data, r.buf, 8);
    udp_send_stream(JbUdp::MsgType::HAND_CMD_ECHO, (const uint8_t*)&p, sizeof(p));
  }
}

void telemetry_step() {
  send_telemetry();
  send_bb_estimates();   // BB pitch/hand pos+vel @ TELEM_RATE_HZ (during-throw diagnostics)
  send_leg_cmd();        // commanded leg interp output @ TELEM_RATE_HZ (float32 interp residual)

  // Stagger the 1 Hz forced refresh across axes so the forced frames never
  // burst together; changed axes are sent immediately regardless of slot.
  static uint32_t tick = 0;
  const uint32_t slot = tick % TELEM_RATE_HZ;              // 0..99, ticks within the second
  const uint32_t slots_per_axis = TELEM_RATE_HZ / NUM_AXES;  // ~14
  const uint64_t now = micros64();   // interval clock: staleness + last_sent_us age
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    const bool due_forced =
        (now - s_base[i].last_sent_us >= DIAG_FORCE_PERIOD_US) &&
        (slot == (uint32_t)i * slots_per_axis);
    // force a Diagnostic on a homing_result transition. Its real value is
    // delivering the RUNNING transition PROMPTLY — independent of the lagging ODrive
    // axis_state — so a normal multi-second home reliably lets the host observer set
    // its saw_running gate. (A same-tick early SETUP-fail goes RUNNING→FAILED inside
    // one homing_step() before the 100 Hz sampler can see RUNNING; the host's timeout
    // backstop catches that — a safe false-FAILURE, never a false-success.)
    // Bench-only: force axis 0's DIAGNOSTIC every telemetry tick (250 Hz) so
    // iq_measured — carried ONLY by the on-change-gated DIAGNOSTIC (>0.5 A / 1 Hz)
    // — is observable at rate for the gain-ladder current-rail / vel-buzz ceilings.
    // Axis 0 is the single bench leg; axes 1-6 keep the on-change/1 Hz cadence:
    // they are absent on the rig (no live iq) and per-axis 250 Hz ×7 would ~2.5× the
    // prio-3 telem-task send load. Folds to a constant-false term in the OFF build.
#if BENCH_SYSID_BUILD
    const bool due_bench = (i == 0);
#else
    constexpr bool due_bench = false;
#endif
    if (diag_changed(axes[i], s_base[i]) ||
        homing_result(i) != s_base[i].homing_result || due_forced || due_bench) {
      send_diag(i);
    }
  }

  // Ball Butler ODrive diag (CAN1 nodes 7/8) — fixed ~1 Hz each, staggered and
  // offset from the platform forced slots (i*slots_per_axis).
  if (slot == 25) send_bb_diag(0);          // bb_pitch → axis_id 7
  else if (slot == 75) send_bb_diag(1);     // bb_hand  → axis_id 8

  ++tick;
}

}  // namespace CanBridge
