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
#include "gpio_poll.h"    // gpio_poll_snapshot() — the hand ball-sensor cache uplinked as HAND_SENSOR
#include "hand_ops.h"     // hand_ops_counters() — per-stage HAND_TRAJ_CMD exits uplinked as BRIDGE_TX_DIAG
#include "clap_link.h"    // clap_tx_stats() — the clapboard downlink census uplinked as CLAP_DIAG

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

// ── Cone uplink: drain cone-bus frames into CONE_FRAME UDP messages ──────────
// Per-tick budget caps the UDP cost if the cone bus ever babbles; legitimate
// cone traffic is ~10-11 fps (10 Hz heartbeat + per-impact catch events)
// against the 100 Hz tick, so the ring (CONE_RING_CAP=16) drains in one frame
// per tick in steady state and a full ring clears in 4 ticks. Excess beyond
// ring+budget is dropped at the ring (counted — can_cone_fwd_drops,
// [canhealth] serial line).
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

// ── Hand ball-sensor uplink — contract in telemetry.h ─────────────────────────
// Freshness is an EQUALITY test on the reply's wall stamp, never an interval
// (time_base.h's clock-discipline invariant) — a wall-anchor step changes the
// stamp, which costs at most one redundant frame.
static constexpr uint64_t HAND_SENSOR_KEEPALIVE_US = 1000000u;   // 1 Hz while no new reply
// The uplink sees every good reply only while the poll is no faster than the
// telemetry tick; a Phase 7 retune below it would silently coalesce raw samples
// (approved decision #4: the raw per-sample bit must survive to ROS).
static_assert(JBBallDetect::CHECK_INTERVAL_MS * TELEM_RATE_HZ >= 1000,
              "hand-sensor poll faster than the telemetry tick — raw samples would coalesce");
static uint64_t s_hand_sensor_sent_wall_us = 0;   // t_bridge_us of the last uplinked reply (0 ⇒ none)
static uint8_t  s_hand_sensor_sent_flags   = 0;   // flags of the last frame (a flip is news)
static uint64_t s_hand_sensor_sent_us      = 0;   // micros64() of the last frame (keepalive pacing)

void hand_sensor_uplink_step() {
  GpioPollSnapshot s;
  gpio_poll_snapshot(s);
  const uint64_t now = micros64();   // interval clock: keepalive pacing
  namespace HSF = JbUdp::HandSensorFlags;
  const uint8_t flags = (uint8_t)((s.raw_held     ? HSF::RAW_HELD       : 0u) |
                                  (s.held         ? HSF::DEBOUNCED_HELD : 0u) |
                                  (s.valid        ? HSF::VALID          : 0u) |
                                  (s.stale        ? HSF::STALE          : 0u) |
                                  (s.time_synced  ? HSF::TIME_SYNCED    : 0u));
  // A flags change is news too: stale/valid flip at read time while the wall
  // stamp is frozen (that is what makes them flip), so keying on the stamp
  // alone would hold a dead sensor's last verdict on the Jetson for up to a
  // keepalive period (~1.24 s worst case) after the bridge already knows.
  const bool fresh = (s.t_bridge_us != s_hand_sensor_sent_wall_us) ||
                     (flags != s_hand_sensor_sent_flags);
  if (!fresh && (now - s_hand_sensor_sent_us) < HAND_SENSOR_KEEPALIVE_US) return;

  JbUdp::HandSensorPayload p{};
  p.t_bridge_us = s.t_bridge_us;   // wire-bound absolute timestamp — wall by contract
  p.raw_states  = s.raw_states;
  p.flags       = flags;
  p.miss_count  = s.miss_count;
  udp_send_stream(JbUdp::MsgType::HAND_SENSOR, (const uint8_t*)&p, sizeof(p));
  s_hand_sensor_sent_wall_us = s.t_bridge_us;
  s_hand_sensor_sent_flags   = flags;
  s_hand_sensor_sent_us      = now;
}

// ── Jugglebot-bus wire-error uplink (the can3_errors row) — contract in telemetry.h ──
// Unconditional 1 Hz, NOT on-change: the whole point is a continuous baseline the
// operator can difference across an A/B (poller on vs off), and "silence means
// healthy" is exactly the ambiguity that cost the 2026-07-29 investigation a
// session. A flat 58 B/s is a rounding error against the ~333 frame/s uplink.
static constexpr uint64_t CAN_ERRORS_PERIOD_US = 1000000u;   // 1 Hz
static uint64_t s_can_errors_sent_us = 0;

void can_errors_uplink_step() {
  const uint64_t now = micros64();   // interval clock: emission pacing
  if (s_can_errors_sent_us != 0 && (now - s_can_errors_sent_us) < CAN_ERRORS_PERIOD_US) return;
  s_can_errors_sent_us = now;

  const CanRxHealth h = can_buses_rx_health();
  const BusRxHealth& j = h.jugglebot;     // jugglebot bus only — see the summary
  JbUdp::CanErrorsPayload p{};
  p.ack_cnt       = j.ack_cnt;
  p.crc_cnt       = j.crc_cnt;
  p.form_cnt      = j.form_cnt;
  p.stuff_cnt     = j.stuff_cnt;
  p.bit0_cnt      = j.bit0_cnt;
  p.bit1_cnt      = j.bit1_cnt;
  p.err_tx_ctx    = j.err_tx_ctx;
  p.err_rx_ctx    = j.err_rx_ctx;
  p.tec_inc_sum   = j.tec_inc_sum;
  p.rec_inc_sum   = j.rec_inc_sum;
  p.tx_gated      = j.tx_gated;
  p.tec_live      = j.tec_live;
  p.rec_live      = j.rec_live;
  p.flt_live      = j.flt_live;
  p.flt_sustained = j.flt_sustained;
  udp_send_stream(JbUdp::MsgType::CAN_ERRORS, (const uint8_t*)&p, sizeof(p));
}

// ── TX-pressure + hand-stage attribution uplink — contract in telemetry.h ────
// Unconditional 1 Hz, NOT on-change, for the same reason as the CanErrors step
// above: the operator reads these by differencing two captures, and silence
// would be ambiguous with health. ALL THREE buses carry tx_deferred + tx_q_hwm,
// unlike CanErrors' deliberate CAN3-only scope — that frame's per-bus cost was
// 15 fields, this one's is 2, so the argument that killed a per-bus array there
// does not apply here.
static constexpr uint64_t BRIDGE_TX_DIAG_PERIOD_US = 1000000u;   // 1 Hz
static uint64_t s_bridge_tx_diag_sent_us = 0;

void bridge_tx_diag_uplink_step() {
  const uint64_t now = micros64();   // interval clock: emission pacing
  if (s_bridge_tx_diag_sent_us != 0 &&
      (now - s_bridge_tx_diag_sent_us) < BRIDGE_TX_DIAG_PERIOD_US) return;
  s_bridge_tx_diag_sent_us = now;

  const CanRxHealth h = can_buses_rx_health();
  const HandOpsCounters hc = hand_ops_counters();
  JbUdp::BridgeTxDiagPayload p{};
  p.tx_deferred_jb   = h.jugglebot.tx_deferred;
  p.tx_deferred_bb   = h.bb.tx_deferred;
  p.tx_deferred_cone = h.cone.tx_deferred;
  p.tx_q_hwm_jb      = h.jugglebot.tx_q_hwm;
  p.tx_q_hwm_bb      = h.bb.tx_q_hwm;
  p.tx_q_hwm_cone    = h.cone.tx_q_hwm;
  p.hand_calls       = hc.calls;
  p.hand_rej_homing  = hc.rej_homing;
  p.hand_bus_down    = hc.bus_down;
  p.hand_pre1_fail   = hc.pre1_fail;
  p.hand_pre2_fail   = hc.pre2_fail;
  p.hand_traj_fail   = hc.traj_fail;
  udp_send_stream(JbUdp::MsgType::BRIDGE_TX_DIAG, (const uint8_t*)&p, sizeof(p));
}

// ── Encoder-cache freshness census — contract in telemetry.h ────────────────
// The confirmation instrument for the surviving question of
// logbook/2026-07-18-teensy-uptime-tracking-degradation.md, after the S1
// experiment localized the uptime command-latency drift to the Teensy with the
// transport, the interp deadline, the heap and the ODrives all exonerated: is
// axes[i].pos_timestamp_us — the encoder cache the leg_interp lead clamp
// measures `fb` against — going STALE with uptime, or does the leg genuinely
// trail? /robot_state and /leg_cmd_executed both read that same cache, so
// nothing already on the wire can tell those apart. This measures the cache's
// freshness directly.
//
// DIAGNOSTICS ONLY: nothing in the firmware reads any state below, so a wrong
// value here cannot move a leg.
static constexpr uint64_t CACHE_DIAG_PERIOD_US = 1000000u;    // 1 Hz emit
static constexpr uint32_t CACHE_AGE_SAT_US     = 0xFFFFFFFFu; // saturation rail (~71.6 min)

// The wire array widths and the firmware's axis count must agree, or the census
// would silently report six legs and a garbage seventh (or overrun the payload).
// All three per-axis arrays are checked: they are read TOGETHER per axis (an age
// against that axis's frame count), so a width that drifted on one of them alone
// would misalign the very comparison the frame exists to support.
static_assert(sizeof(JbUdp::CacheDiagPayload::age_min_us) /
                  sizeof(JbUdp::CacheDiagPayload::age_min_us[0]) == NUM_AXES,
              "CacheDiag age_min array width != NUM_AXES");
static_assert(sizeof(JbUdp::CacheDiagPayload::age_max_us) /
                  sizeof(JbUdp::CacheDiagPayload::age_max_us[0]) == NUM_AXES,
              "CacheDiag age_max array width != NUM_AXES");
static_assert(sizeof(JbUdp::CacheDiagPayload::enc_frames) /
                  sizeof(JbUdp::CacheDiagPayload::enc_frames[0]) == NUM_AXES,
              "CacheDiag enc_frames array width != NUM_AXES");

// Window accumulators. ALL of these are touched by task_telem and nothing else.
static uint64_t s_cache_diag_sent_us = 0;   // micros64() at the last emit (0 ⇒ none yet)
static uint32_t s_cache_diag_seq     = 0;   // frames emitted since boot
static uint32_t s_age_min_us[NUM_AXES];     // valid only for axes whose seen bit is set —
static uint32_t s_age_max_us[NUM_AXES];     // an axis's first REAL fold in a window seeds BOTH
static uint16_t s_age_samples        = 0;
static uint8_t  s_age_seen_mask      = 0;

void cache_diag_uplink_step() {
  const uint64_t now = micros64();   // interval clock: ages + emission pacing.
                                     // NEVER now_wall_us — an anchor step would
                                     // underflow an age into a huge "fresh" number.

  // ── Fold ONE sample per call (TELEM_RATE_HZ) into the open window ──────────
  // snapshot_pos_vel is axis_state.h's seqlock reader — the SAME reader
  // send_telemetry() already uses on this exact triple at this exact rate. It is
  // the right atomic discipline here: the writer (can_buses.cpp
  // decode_into_cache, on the PRIORITY-5 task_can_rx) preempts this priority-3
  // task, and pos_timestamp_us is a u64, i.e. two 32-bit loads on Cortex-M7 that
  // a preemption can tear. Unlike atomic_read_u64 the seqlock achieves that
  // WITHOUT masking interrupts, so this census cannot delay the 500 Hz interp
  // ISR by even one instruction — which is what makes "instrumentation only,
  // no control-path effect" a structural property rather than a claim.
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    float pos, vel; uint64_t ts;
    snapshot_pos_vel(axes[i], pos, vel, ts);
    if (ts == 0) {
      // Never cached (absent axis / pre-first-frame): fold NOTHING. The emit
      // step paints the rail into min/max for any axis whose seen bit is still
      // clear — so "no data" and "catastrophically stale" stay different
      // answers, AND a first frame landing mid-window cannot leave a rail from
      // earlier ts==0 folds inside a window whose seen bit ends up set.
      continue;
    }
    const uint8_t bit = (uint8_t)(1u << i);
    // ts can EXCEED now: the priority-5 writer (task_can_rx) may stamp between
    // our micros64() capture above and this snapshot — a seqlock retry returns
    // the NEWER timestamp. That sample is maximally FRESH; clamp its age to 0.
    // An unclamped subtraction would wrap to ~2^64 and paint the saturation
    // rail into age_max — a spurious "stale" spike in the exact instrument the
    // S2 soak reads.
    const uint64_t d = (ts <= now) ? (now - ts) : 0;
    const uint32_t age =
        (d > (uint64_t)CACHE_AGE_SAT_US) ? CACHE_AGE_SAT_US : (uint32_t)d;
    if (!(s_age_seen_mask & bit)) {
      s_age_seen_mask |= bit;   // first REAL sample this window seeds both extrema
      s_age_min_us[i] = age;
      s_age_max_us[i] = age;
    } else {
      if (age < s_age_min_us[i]) s_age_min_us[i] = age;
      if (age > s_age_max_us[i]) s_age_max_us[i] = age;
    }
  }
  if (s_age_samples != 0xFFFFu) ++s_age_samples;   // saturate; window_us stays honest

  // ── Emit once the window is full ──────────────────────────────────────────
  // The fold above ran FIRST, so the emit-instant read is this window's LAST
  // sample: the frame is never built from a window that excludes its own
  // timestamp.
  if (s_cache_diag_sent_us != 0 &&
      (now - s_cache_diag_sent_us) < CACHE_DIAG_PERIOD_US) return;
  const uint64_t win64 = (s_cache_diag_sent_us == 0) ? 0ULL : (now - s_cache_diag_sent_us);
  s_cache_diag_sent_us = now;
  ++s_cache_diag_seq;

  const CanRxHealth h = can_buses_rx_health();
  JbUdp::CacheDiagPayload p{};
  p.t_local_us = now;
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    // Rail for axes with no real sample this window: their extrema statics hold
    // stale values from an earlier window (or nothing), never data from this one.
    const bool seen = (s_age_seen_mask & (uint8_t)(1u << i)) != 0;
    p.age_min_us[i] = seen ? s_age_min_us[i] : CACHE_AGE_SAT_US;
    p.age_max_us[i] = seen ? s_age_max_us[i] : CACHE_AGE_SAT_US;
    // Cumulative since boot, NOT differenced here: the consumer differences two
    // frames, and `seq` is what tells it whether the two are adjacent. Doing the
    // subtraction on-chip would hide a dropped frame inside a plausible-looking
    // count (the BridgeTxDiag census idiom, and the opposite of ClockDiag's
    // occupancy fields — those are differenced on-chip because their window is
    // the ANCHOR interval, which the host cannot reconstruct).
    p.enc_frames[i] = h.enc_frames[i];
  }
  p.seq                = s_cache_diag_seq;
  p.window_us          = (win64 > (uint64_t)CACHE_AGE_SAT_US)
                             ? CACHE_AGE_SAT_US : (uint32_t)win64;
  // Bus fields are addressed by ROLE (jugglebot / bb / cone), matching
  // BridgeTxDiag. The jugglebot and cone CONTROLLERS are currently swapped in
  // hardware (can_buses.cpp), so a controller-numbered name would be wrong the
  // moment that swap is undone.
  p.rx_cap_hits_jb     = h.jugglebot.cap_hits;
  p.rx_cap_hits_bb     = h.bb.cap_hits;
  p.rx_cap_hits_cone   = h.cone.cap_hits;
  p.decode_short       = h.decode_short;
  p.decode_bad_axis    = h.decode_bad_axis;
  p.rx_depth_hwm_jb    = h.jugglebot.depth_hwm;
  p.rx_depth_hwm_bb    = h.bb.depth_hwm;
  p.rx_depth_hwm_cone  = h.cone.depth_hwm;
  p.samples            = s_age_samples;
  p.seen_mask          = s_age_seen_mask;
  udp_send_stream(JbUdp::MsgType::CACHE_DIAG, (const uint8_t*)&p, sizeof(p));

  // Close the window AFTER the payload is built, and unconditionally — the send
  // result is deliberately ignored. Holding the extrema back on a dropped frame
  // would silently merge two windows and manufacture a peak; a lost frame is
  // already visible as a gap in `seq`. Zeroing the seen mask re-arms the
  // per-axis seeding above, so the extrema need no sentinel value.
  s_age_samples   = 0;
  s_age_seen_mask = 0;
}

// ── CAN RX-ring true-occupancy census — contract in telemetry.h ─────────────
// The conviction instrument for the FlexCAN_T4 `_available` leak. Everything
// this function emits was reduced where the data lives (can_buses.cpp's 1 kHz
// service tick, the jugglebot RX decode, gpio_poll's request/reply pair); the
// step below only takes the snapshots and turns cumulative counters into window
// quantities.
//
// DIAGNOSTICS ONLY: nothing in the firmware reads any state below, and the leak
// itself is deliberately NOT fixed in this build — the fix is sequenced after
// the measurement so it can be judged against a number instead of a theory.
static constexpr uint64_t RING_DIAG_PERIOD_US = 1000000u;   // 1 Hz emit
static constexpr uint32_t RING_U32_SAT        = 0xFFFFFFFFu;

// Window state. ALL of these are touched by task_telem and nothing else.
static uint64_t s_ring_diag_sent_us   = 0;   // micros64() at the last emit (0 ⇒ none yet)
static uint32_t s_ring_diag_seq       = 0;   // frames emitted since boot
static uint32_t s_ring_ticks_prev     = 0;   // previous take's cumulative probe_ticks
static uint64_t s_ring_arrival_prev   = 0;   // previous take's arrival clock
static uint32_t s_ring_lag_frames_prev = 0;  // previous take's cumulative folded-frame count

void ring_diag_uplink_step() {
  const uint64_t now = micros64();   // interval clock: emission pacing.
                                     // NEVER now_wall_us — an anchor step would
                                     // corrupt the window length and, with it,
                                     // the decode-vs-capture span comparison.
  if (s_ring_diag_sent_us != 0 &&
      (now - s_ring_diag_sent_us) < RING_DIAG_PERIOD_US) return;
  const uint64_t win64 = (s_ring_diag_sent_us == 0) ? 0ULL : (now - s_ring_diag_sent_us);
  s_ring_diag_sent_us = now;
  ++s_ring_diag_seq;

  const CanRingProbe rp  = can_buses_ring_probe();
  const JbLagProbe   lag = can_buses_jb_lag_take();
  GpioPollRtt rtt{};
  gpio_poll_rtt_take(rtt);

  JbUdp::RingDiagPayload p{};
  p.t_local_us = now;
  // Bus fields are addressed by ROLE (jugglebot / bb / cone), matching
  // BridgeTxDiag and CacheDiag. The jugglebot and cone CONTROLLERS are currently
  // swapped in hardware (can_buses.cpp), so a controller-numbered name would be
  // wrong the moment that swap is undone.
  p.true_depth_jb          = rp.jugglebot.depth;
  p.true_depth_bb          = rp.bb.depth;
  p.true_depth_cone        = rp.cone.depth;
  p.avail_reported_jb      = rp.jugglebot.avail;
  p.avail_reported_bb      = rp.bb.avail;
  p.avail_reported_cone    = rp.cone.avail;
  p.leak_hwm_jb            = rp.jugglebot.leak_hwm;
  p.leak_hwm_bb            = rp.bb.leak_hwm;
  p.leak_hwm_cone          = rp.cone.leak_hwm;
  p.true_depth_hwm_jb      = rp.jugglebot.depth_hwm;
  p.true_depth_hwm_bb      = rp.bb.depth_hwm;
  p.true_depth_hwm_cone    = rp.cone.depth_hwm;
  p.fifo_overflows_jb      = rp.jugglebot.fifo_overflows;
  p.fifo_overflows_bb      = rp.bb.fifo_overflows;
  p.fifo_overflows_cone    = rp.cone.fifo_overflows;
  p.fifo_warns_jb          = rp.jugglebot.fifo_warns;
  p.fifo_warns_bb          = rp.bb.fifo_warns;
  p.fifo_warns_cone        = rp.cone.fifo_warns;

  // ── Cumulative → per-window, differenced HERE ─────────────────────────────
  // The opposite choice from CacheDiag's enc_frames (cumulative on the wire,
  // differenced by the host) and the same one ClockDiag's occupancy fields make,
  // for the same reason: these three are denominators for extrema that were
  // ALREADY reduced on-chip over this exact window. A host-side difference
  // would silently widen the denominator across a dropped uplink frame while the
  // numerator stayed one window's worth, understating a duty. Unsigned
  // differencing is wrap-correct.
  p.probe_ticks = rp.probe_ticks - s_ring_ticks_prev;
  s_ring_ticks_prev = rp.probe_ticks;
  p.lag_frames = lag.frames - s_ring_lag_frames_prev;
  s_ring_lag_frames_prev = lag.frames;
  // The arrival clock RESTARTS at a reseed, so an unguarded difference would go
  // hugely negative (and, unsigned, enormous) exactly when the series is least
  // trustworthy. A reseeded window reports 0 span and says so in the flags.
  const uint64_t cap64 = (lag.reseeded_since_read || lag.arrival_us < s_ring_arrival_prev)
                             ? 0ULL : (lag.arrival_us - s_ring_arrival_prev);
  s_ring_arrival_prev = lag.arrival_us;
  p.cap_span_us = (cap64 > (uint64_t)RING_U32_SAT) ? RING_U32_SAT : (uint32_t)cap64;

  p.lag_now_us  = lag.seeded ? lag.lag_now_us : 0;
  p.lag_hwm_us  = lag.seeded ? lag.lag_hwm_us : 0;
  p.lag_reseeds = lag.reseeds;
  p.flags = (uint8_t)((lag.seeded ? JbUdp::RingDiagFlags::LAG_SEEDED : 0u) |
                      (lag.reseeded_since_read
                           ? JbUdp::RingDiagFlags::LAG_RESEED_IN_WINDOW : 0u));

  p.sdo_rtt_min_us  = rtt.min_us;
  p.sdo_rtt_max_us  = rtt.max_us;
  p.sdo_rtt_last_us = rtt.last_us;
  p.sdo_rtt_count   = rtt.count;

  p.seq       = s_ring_diag_seq;
  p.window_us = (win64 > (uint64_t)RING_U32_SAT) ? RING_U32_SAT : (uint32_t)win64;
  udp_send_stream(JbUdp::MsgType::RING_DIAG, (const uint8_t*)&p, sizeof(p));
  // The send result is deliberately ignored and the window baselines above were
  // already advanced: holding them back on a dropped frame would merge two
  // windows and inflate a duty, while a lost frame is already visible as a gap
  // in `seq` (the CacheDiag precedent).
}

// ── Firmware identity uplink — contract in telemetry.h ───────────────────────
// Both fields are compile-time constants, so this frame never changes within a
// boot; it is sent at 1 Hz anyway because the bridge cannot observe a Jetson
// attaching, and 13 B/s buys "the host always knows what it is talking to"
// within one second of any start order.
static constexpr uint64_t BRIDGE_IDENTITY_PERIOD_US = 1000000u;   // 1 Hz
static uint64_t s_bridge_identity_sent_us = 0;

void bridge_identity_uplink_step() {
  const uint64_t now = micros64();   // interval clock: emission pacing
  if (s_bridge_identity_sent_us != 0 &&
      (now - s_bridge_identity_sent_us) < BRIDGE_IDENTITY_PERIOD_US) return;
  s_bridge_identity_sent_us = now;

  JbUdp::BridgeIdentityPayload p{};
  p.fw_version       = FW_VERSION;
  p.protocol_version = JbUdp::PROTOCOL_VERSION;
  udp_send_stream(JbUdp::MsgType::BRIDGE_IDENTITY, (const uint8_t*)&p, sizeof(p));
}

// ── Clapboard downlink census uplink — contract in telemetry.h ───────────────
static constexpr uint64_t CLAP_DIAG_PERIOD_US = 1000000u;   // 1 Hz
static uint64_t s_clap_diag_sent_us = 0;

void clap_diag_uplink_step() {
  const uint64_t now = micros64();   // interval clock: emission pacing
  if (s_clap_diag_sent_us != 0 &&
      (now - s_clap_diag_sent_us) < CLAP_DIAG_PERIOD_US) return;
  s_clap_diag_sent_us = now;

  const ClapTxStats t = clap_tx_stats();
  JbUdp::ClapDiagPayload p{};
  p.queued   = t.queued;
  p.sent     = t.sent;
  p.gated    = t.gated;
  p.dropped  = t.dropped;
  p.ring_hwm = t.ring_hwm;
  udp_send_stream(JbUdp::MsgType::CLAP_DIAG, (const uint8_t*)&p, sizeof(p));
  // Send result deliberately ignored: the counters are CUMULATIVE, so a dropped
  // frame costs one sample of resolution and never a lost event (the CacheDiag /
  // RingDiag precedent — nothing here is a per-window difference computed on the
  // Teensy).
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
