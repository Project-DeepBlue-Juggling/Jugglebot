// =============================================================================
//  can_buses.cpp — three subsystem-isolated FlexCAN_T4 buses + RX decode + TX
// =============================================================================
#include "can_buses.h"

#include <FlexCAN_T4.h>
#include "canbridge_config.h"
#include "protocol_config.h"
#include "axis_state.h"
#include "ball_butler_state.h"
#include "time_base.h"
#include "odrive_protocol.h"
#include "version_check.h"   // cache Get_Version replies (version_record)

namespace CanBridge {

// Peripheral identity is confined to these three lines (ADR-0013):
//   bb = CAN1 (pins 22/23), cone = CAN2 (pins 1/0), jugglebot = CAN3 (pins 31/30).
// TX_SIZE_64 (was 16): the software TX ring the TX-complete ISR drains. A 16-deep
// ring silently dropped frames on dense bursts — a multi-leg config burst lost most
// set_state(CLOSED_LOOP) frames so the legs never armed (2026-06-26). 64 absorbs any
// realistic burst (worst case ~30 frames for a six-leg activate) with headroom; the
// callers also avoid bursting (one leg/tick), so this is defence-in-depth. ~1 KB
// RAM/bus, trivial against the ~900 KB headroom.
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> can_bb;
static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_64> can_cone;
static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_64> can_jugglebot;

static volatile uint32_t s_bb_rx = 0, s_bb_tx = 0, s_cone_rx = 0, s_cone_tx = 0,
                         s_jugglebot_rx = 0, s_jugglebot_tx = 0;
static volatile uint64_t s_bb_last_rx_us = 0, s_cone_last_rx_us = 0, s_jugglebot_last_rx_us = 0;

// RX-health observability counters (see can_buses.h "RX-health observability").
// Single-writer: task_can_rx — the service loop (depth/cap/bus-error fields) and the
// decode callback (decode_* fields). EXCEPTION: tx_gated is written by the TX gate in
// partner_recent() from sender contexts (interp ISR + tasks) under a PRIMASK-masked
// increment. (One boot-window exception: before the first
// events() call flips the library's isEventsUsed, the FlexCAN ISR dispatches the decode
// callback directly, so the decode_* increments briefly run in ISR context — harmless,
// since task_diag is not reading yet.) Each field is one word (atomic load/store on
// Cortex-M7) and cumulative/sticky; err_flags is a producer-local read-or-store. The
// cross-task read in task_diag therefore needs no lock and no reset drops a transient.
static volatile BusRxHealth s_bb_rxh{}, s_cone_rxh{}, s_jugglebot_rxh{};
static volatile uint32_t s_decode_short = 0, s_decode_bad_axis = 0;

// Hand command-echo recorder. Defined later in this TU (near the ring
// helpers); forward-declared here so decode_into_cache can stash a sniffed hand
// Set_Input_Pos into the single-slot latest-value store.
static inline void hand_cmd_echo_record(const uint8_t* d, uint64_t now);

// Decode an ODrive frame into the per-axis cache. Used by the Jugglebot bus only
// (CAN3 carries all 6 leg ODrives + the Hand ODrive).
static void decode_into_cache(const CAN_message_t& msg) {
  const uint8_t axis = ODrive::axis_of(msg.id);
  const uint8_t cmd  = ODrive::cmd_of(msg.id);
  if (axis >= NUM_AXES) { s_decode_bad_axis++; return; }   // node id we don't cache (stray/garbled)
  if (msg.len < 8) { s_decode_short++; return; }           // truncated (mirrors odrive.py _check_len;
                                                           // every ODrive telemetry frame we decode is DLC 8)
  AxisState& a = axes[axis];
  const uint8_t* d = msg.buf;

  switch (cmd) {
    case ODriveCmd::heartbeat_message: {
      auto h = ODrive::decode_heartbeat(d);
      a.axis_state       = h.state;
      a.procedure_result = h.procedure_result;
      a.trajectory_done  = h.trajectory_done;
      atomic_write_u64(&a.last_heartbeat_us, micros64());   // monotonic; atomic 64-bit:
                                          // the u64 write is two 32-bit stores on Cortex-M7; a lower-prio
                                          // reader (fault watchdog) mid-load would tear the timestamp
      a.heartbeat_seen   = true;
      a.heartbeat_stale  = false;
      break;
    }
    case ODriveCmd::get_error: {
      auto e = ODrive::decode_error(d);
      a.active_errors = e.active_errors;
      a.disarm_reason = e.disarm_reason;
      break;
    }
    case ODriveCmd::get_encoder_estimate: {
      auto p = ODrive::decode_encoder_estimate(d);   // (pos, vel) in ODrive convention
      // Sign-flip legs to the Jugglebot convention (positive = extension),
      // mirroring can_node._handle_encoder's _leg_sign.
      const float pos = ODrive::leg_sign(axis, p.a);
      const float vel = ODrive::leg_sign(axis, p.b);
      write_pos_vel(a, pos, vel, micros64());   // pos_timestamp_us monotonic: MOTOR_FB_STALE reads it as an interval
      break;
    }
    case ODriveCmd::get_iq: {
      auto q = ODrive::decode_iq(d);
      a.iq_setpoint = q.a; a.iq_measured = q.b;
      break;
    }
    case ODriveCmd::get_temps: {
      auto t = ODrive::decode_temps(d);
      a.temp_fet = t.a; a.temp_motor = t.b;
      break;
    }
    case ODriveCmd::get_bus_voltage_current: {
      auto v = ODrive::decode_bus_voltage_current(d);
      a.bus_voltage = v.a; a.bus_current = v.b;
      break;
    }
    case ODriveCmd::get_version:
      // cache the raw 8-byte Get_Version payload for the bridge's
      // GET_AXIS_VERSIONS pull (version SEMANTICS stay on the Jetson). The reply
      // is DLC 8 (the empty-payload request we sent is len 0 and was already
      // dropped by the `< 8` guard above), so `d` is a full 8-byte version frame.
      version_record(axis, d);
      break;
    case ODriveCmd::set_input_pos:
      // sniff the Platform Teensy's Set_Input_Pos to the HAND ODrive
      // (axis 6) and echo it to the host (HAND_CMD_ECHO → hand_telemetry
      // pos_cmd/vel_ff_cmd/tor_ff_cmd, can_node._handle_hand_input_pos parity).
      // CAN3 SRX_DIS means we never receive our own leg-setpoint TX, so only
      // genuine Platform→hand commands reach here; the axis==HAND_AXIS guard is
      // belt-and-suspenders (and skips any stray leg-addressed set_input_pos).
      // wire-bound absolute timestamp — wall by contract: t_bridge_us is
      // serialised into the HAND_CMD_ECHO uplink for host-side wall-clock correlation.
      if (axis == HAND_AXIS) hand_cmd_echo_record(d, now_wall_us());
      break;
    // TxSdo handled elsewhere (encoder-search). Ignore here.
    default:
      break;
  }
}

// Decode a Ball Butler heartbeat (CAN1 id 0x7D1) into bb_state. Frame layout
// (8 bytes, little-endian; mirrors ball_butler.py BallButlerHeartbeat.from_can_frame):
//   byte 0 = ball_in_hand (bit 0) | state (bits 1..7)
//   byte 1 = state_data (error code when state == ERROR, else 0)
//   bytes 2-3 = yaw_enc  (uint16, scaled by HeartbeatEncoding::yaw_res_deg)
//   bytes 4-5 = pitch_enc(uint16, scaled by HeartbeatEncoding::pitch_res_deg)
//   bytes 6-7 = hand_enc (uint16, scaled by HeartbeatEncoding::hand_res_mm)
// Short frames (< 8 bytes) are silently dropped — bb_state retains its last value.
static void decode_bb_heartbeat(const CAN_message_t& msg) {
  if (msg.len < 8) return;
  const uint8_t* d = msg.buf;
  uint16_t yaw_enc, pitch_enc, hand_enc;
  memcpy(&yaw_enc,   &d[2], 2);
  memcpy(&pitch_enc, &d[4], 2);
  memcpy(&hand_enc,  &d[6], 2);
  write_bb_heartbeat(
      bb_state,
      /*ball_in_hand=*/ (d[0] & 0x01u) != 0,
      /*state=*/        (uint8_t)(d[0] >> 1),
      /*state_data=*/   d[1],
      /*yaw_deg=*/      (float)yaw_enc   * HeartbeatEncoding::yaw_res_deg,
      /*pitch_deg=*/    (float)pitch_enc * HeartbeatEncoding::pitch_res_deg,
      /*hand_mm=*/      (float)hand_enc  * HeartbeatEncoding::hand_res_mm,
      /*t_us=*/         micros64());   // bb_state.last_heartbeat_us monotonic: read as an interval by the watchdog + [bb] diag
}

// Ball Butler ODrive telemetry on CAN1 (node ids 7=pitch, 8=hand). Decode the
// slow telemetry frames into the separate bb_axes cache — emitted as DIAGNOSTIC
// frames (axis_id 7/8) by telemetry.cpp. Mirrors the platform decode_into_cache
// switch but writes bb_axes; the platform path (CAN3 → axes[]) is untouched.
static void decode_bb_odrive(const CAN_message_t& msg) {
  const uint8_t node = ODrive::axis_of(msg.id);
  if (node < BB_FIRST_NODE || node >= (uint8_t)(BB_FIRST_NODE + NUM_BB_AXES)) return;
  if (msg.len < 8) return;
  AxisState& a = bb_axes[node - BB_FIRST_NODE];
  const uint8_t* d = msg.buf;
  switch (ODrive::cmd_of(msg.id)) {
    case ODriveCmd::heartbeat_message: {
      auto h = ODrive::decode_heartbeat(d);
      a.axis_state = h.state; a.procedure_result = h.procedure_result;
      a.trajectory_done = h.trajectory_done;
      atomic_write_u64(&a.last_heartbeat_us, micros64()); a.heartbeat_seen = true; a.heartbeat_stale = false;  // monotonic; atomic 64-bit: torn-write guard, read as interval by send_bb_diag
      break;
    }
    case ODriveCmd::get_error: {
      auto e = ODrive::decode_error(d);
      a.active_errors = e.active_errors; a.disarm_reason = e.disarm_reason;
      break;
    }
    case ODriveCmd::get_encoder_estimate: {
      auto p = ODrive::decode_encoder_estimate(d);
      write_pos_vel(a, p.a, p.b, micros64());   // BB is not a leg → no sign flip; pos_timestamp_us monotonic
      break;
    }
    case ODriveCmd::get_iq: {
      auto q = ODrive::decode_iq(d);
      a.iq_setpoint = q.a; a.iq_measured = q.b;
      break;
    }
    case ODriveCmd::get_temps: {
      auto t = ODrive::decode_temps(d);
      a.temp_fet = t.a; a.temp_motor = t.b;
      break;
    }
    case ODriveCmd::get_bus_voltage_current: {
      auto v = ODrive::decode_bus_voltage_current(d);
      a.bus_voltage = v.a; a.bus_current = v.b;
      break;
    }
    default: break;
  }
}

// ── BB command-result uplink ring (CAN1 CMD_RESULT → UDP relay) ──────
// SPSC mirror of the cone ring: producer is on_bb_rx (task_can_rx context),
// consumer is cmd_result_uplink_step() on task_telem. Both sides mask IRQs for the
// few hundred ns of the copy. CMD_RESULT is low-rate (one per operator command),
// so a small ring is ample; sustained overflow would mean BB babble, counted in
// s_cmd_result_fwd_drops (drop-OLDEST) and surfaced on [canhealth].
static constexpr uint8_t CMD_RESULT_RING_CAP = 8;
static CmdResultFrameRec s_cmd_result_ring[CMD_RESULT_RING_CAP];
static volatile uint8_t s_cmd_result_ring_head = 0, s_cmd_result_ring_tail = 0;
static volatile uint32_t s_cmd_result_fwd_drops = 0;

// Copy a CMD_RESULT frame verbatim into the uplink ring. Drop-OLDEST on overflow
// CMD_RESULT is the LOUD operator command-outcome channel, so under a
// burst the operator wants the FRESHEST results — the prior drop-newest policy
// discarded exactly the latest outcome they were waiting on. On a full ring, evict
// the oldest record (advance the tail), then enqueue. Touching the tail from the
// producer is safe because this push and can_cmd_result_pop are mutually exclusive
// under the shared PRIMASK critical section (both mask IRQs on this single-core M7 —
// not a lock-free head/tail split, so there is no producer/consumer tail race).
static inline void cmd_result_ring_push(const CAN_message_t& msg, uint64_t now) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const uint8_t next = (uint8_t)((s_cmd_result_ring_head + 1) % CMD_RESULT_RING_CAP);
  if (next == s_cmd_result_ring_tail) {                  // ring full → drop OLDEST
    s_cmd_result_ring_tail = (uint8_t)((s_cmd_result_ring_tail + 1) % CMD_RESULT_RING_CAP);
    s_cmd_result_fwd_drops++;
  }
  CmdResultFrameRec& r = s_cmd_result_ring[s_cmd_result_ring_head];
  r.t_bridge_us = now;
  r.can_id = msg.id;
  r.dlc = (msg.len > 8) ? 8 : msg.len;
  memcpy(r.buf, msg.buf, 8);
  s_cmd_result_ring_head = next;
  __set_PRIMASK(pm);
}

bool can_cmd_result_pop(CmdResultFrameRec& out) {
  bool have = false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  if (s_cmd_result_ring_tail != s_cmd_result_ring_head) {
    out = s_cmd_result_ring[s_cmd_result_ring_tail];
    s_cmd_result_ring_tail = (uint8_t)((s_cmd_result_ring_tail + 1) % CMD_RESULT_RING_CAP);
    have = true;
  }
  __set_PRIMASK(pm);
  return have;
}

uint32_t can_cmd_result_fwd_drops() { return s_cmd_result_fwd_drops; }

// CAN1 Ball Butler: the BB heartbeat (0x7D1) decodes into bb_state; the CMD_RESULT
// (0x7D5) loud-channel frame is relayed verbatim to the host; BB pitch/hand ODrive
// telemetry (nodes 7/8) decodes into bb_axes. Everything else is counted via
// s_bb_rx and dropped.
static void on_bb_rx(const CAN_message_t& msg) {
  s_bb_rx++;
  // DUAL-USE split: the health stamp is an INTERVAL (→ micros64), but the
  // relay-ring t_bridge_us is a WIRE-BOUND absolute timestamp copied into the
  // CMD_RESULT uplink (→ wall). Keep them on separate clocks.
  const uint64_t now      = now_wall_us();   // wire-bound absolute timestamp — wall by contract
  const uint64_t now_mono = micros64();      // interval clock for the health stamp
  atomic_write_u64(&s_bb_last_rx_us, now_mono);   // 64-bit monotonic; read as an interval by health_of
  if (msg.id == BallButlerCanId::HEARTBEAT)  { decode_bb_heartbeat(msg); return; }
  if (msg.id == BallButlerCanId::CMD_RESULT) { cmd_result_ring_push(msg, now); return; }
  decode_bb_odrive(msg);   // BB pitch/hand ODrive telemetry (CAN1 nodes 7/8)
}

// ── Cone uplink ring (CAN2 → CONE_FRAME UDP relay) ─────────────────
// SPSC: producer is on_cone_rx (task_can_rx context — briefly the FlexCAN ISR
// during the boot window, see the RX-health comment above), consumer is
// cone_uplink_step() on task_telem. Both sides run their critical section with
// IRQs masked (the can_*_send / atomic_*_u64 idiom), which sidesteps every
// memory-ordering subtlety for a few hundred ns per ~21-byte record. Capacity
// covers a full 100 Hz consumer tick of worst-case legitimate cone traffic
// (10 Hz heartbeat + catch events deferred ~30 ms by the cone's report delay)
// many times over; sustained overflow means CAN2 babble, counted in
// s_cone_fwd_drops (drop-newest) and surfaced on the [canhealth] serial line.
static constexpr uint8_t CONE_RING_CAP = 16;
static ConeFrameRec s_cone_ring[CONE_RING_CAP];
static volatile uint8_t s_cone_ring_head = 0, s_cone_ring_tail = 0;
static volatile uint32_t s_cone_fwd_drops = 0;

// CAN2 catching cone: no ODrive on this bus → count, then copy the frame into
// the cone-uplink ring for the Jetson relay. The RX timestamp also drives the
// cone-presence gate in can_cone_send() (cone-absent tolerance).
static void on_cone_rx(const CAN_message_t& msg) {
  // DUAL-USE split: s_cone_last_rx_us drives the cone-presence gate + health
  // (INTERVAL → micros64); the ring t_bridge_us is a wire-bound absolute timestamp (→ wall).
  const uint64_t now      = now_wall_us();   // wire-bound absolute timestamp — wall by contract
  const uint64_t now_mono = micros64();      // interval clock for the gate + health stamp
  s_cone_rx++;
  atomic_write_u64(&s_cone_last_rx_us, now_mono);   // 64-bit monotonic; read as an interval by the cone gate + health_of
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const uint8_t next = (uint8_t)((s_cone_ring_head + 1) % CONE_RING_CAP);
  if (next == s_cone_ring_tail) {
    s_cone_fwd_drops++;                        // ring full → drop newest
  } else {
    ConeFrameRec& r = s_cone_ring[s_cone_ring_head];
    r.t_bridge_us = now;
    r.can_id = msg.id;
    r.dlc = (msg.len > 8) ? 8 : msg.len;
    memcpy(r.buf, msg.buf, 8);
    s_cone_ring_head = next;
  }
  __set_PRIMASK(pm);
}

bool can_cone_pop(ConeFrameRec& out) {
  bool have = false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  if (s_cone_ring_tail != s_cone_ring_head) {
    out = s_cone_ring[s_cone_ring_tail];
    s_cone_ring_tail = (uint8_t)((s_cone_ring_tail + 1) % CONE_RING_CAP);
    have = true;
  }
  __set_PRIMASK(pm);
  return have;
}

uint32_t can_cone_fwd_drops() { return s_cone_fwd_drops; }

// ── Platform-Teensy relay-reply uplink ring ─────────────────────────
// SPSC mirror of the cone ring: producer is on_jugglebot_rx (task_can_rx), consumer
// is platform_uplink_step() on task_telem. The Platform Teensy answers a relay read
// on the SAME arbitration id it was triggered on (0x6E0 RobotState, 0x7DE tilt), so
// every CAN3 frame whose id is a Platform reply id is copied here verbatim and
// uplinked as a PLATFORM_FRAME for the host to decode + correlate. Replies are
// low-rate (one per operator relay read), so a small ring is ample; sustained
// overflow would mean a babbling Platform partner, counted in s_platform_fwd_drops.
static constexpr uint8_t PLATFORM_RING_CAP = 8;
static PlatformFrameRec s_platform_ring[PLATFORM_RING_CAP];
static volatile uint8_t s_platform_ring_head = 0, s_platform_ring_tail = 0;
static volatile uint32_t s_platform_fwd_drops = 0;

static inline void platform_ring_push(const CAN_message_t& msg, uint64_t now) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const uint8_t next = (uint8_t)((s_platform_ring_head + 1) % PLATFORM_RING_CAP);
  if (next == s_platform_ring_tail) {
    s_platform_fwd_drops++;                    // ring full → drop newest
  } else {
    PlatformFrameRec& r = s_platform_ring[s_platform_ring_head];
    r.t_bridge_us = now;
    r.can_id = msg.id;
    r.dlc = (msg.len > 8) ? 8 : msg.len;
    memcpy(r.buf, msg.buf, 8);
    s_platform_ring_head = next;
  }
  __set_PRIMASK(pm);
}

bool can_platform_pop(PlatformFrameRec& out) {
  bool have = false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  if (s_platform_ring_tail != s_platform_ring_head) {
    out = s_platform_ring[s_platform_ring_tail];
    s_platform_ring_tail = (uint8_t)((s_platform_ring_tail + 1) % PLATFORM_RING_CAP);
    have = true;
  }
  __set_PRIMASK(pm);
  return have;
}

uint32_t can_platform_fwd_drops() { return s_platform_fwd_drops; }

// ── Hand command-echo latest-value slot ─────────────────────────────
// Producer: decode_into_cache (task_can_rx) via hand_cmd_echo_record on a sniffed
// hand Set_Input_Pos. Consumer: hand_cmd_echo_uplink_step (task_telem) via
// can_hand_cmd_echo_pop. Single slot + dirty flag → coalesce to the newest command
// (a diagnostic echo, not a lossless stream). SPSC, PRIMASK-guarded like the ring.
static HandCmdEchoRec s_hand_cmd;
static volatile bool  s_hand_cmd_dirty = false;

static inline void hand_cmd_echo_record(const uint8_t* d, uint64_t now) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s_hand_cmd.t_bridge_us = now;
  memcpy(s_hand_cmd.buf, d, 8);
  s_hand_cmd_dirty = true;
  __set_PRIMASK(pm);
}

bool can_hand_cmd_echo_pop(HandCmdEchoRec& out) {
  bool have = false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  if (s_hand_cmd_dirty) {
    out = s_hand_cmd;
    s_hand_cmd_dirty = false;
    have = true;
  }
  __set_PRIMASK(pm);
  return have;
}

// is_platform_reply_id() is an inline classifier in can_buses.h (shared with the
// native harness without compiling this TU host-side).

// CAN3 Jugglebot core: a Platform-Teensy relay reply (0x6E0 / 0x7DE) is forwarded
// verbatim to the host via the relay ring; every other frame is a leg/hand ODrive
// frame and decodes into the cache. (axis_of(0x6E0)=55 >= NUM_AXES, so before this
// filter the relay replies were silently counted as decode_bad_axis and dropped.)
static void on_jugglebot_rx(const CAN_message_t& msg) {
  s_jugglebot_rx++;
  // DUAL-USE split: s_jugglebot_last_rx_us drives bus health (INTERVAL →
  // micros64); the relay-ring t_bridge_us is a wire-bound absolute timestamp copied
  // into the PLATFORM_FRAME uplink (→ wall).
  const uint64_t now      = now_wall_us();   // wire-bound absolute timestamp — wall by contract
  const uint64_t now_mono = micros64();      // interval clock for the health stamp
  atomic_write_u64(&s_jugglebot_last_rx_us, now_mono);   // 64-bit monotonic; read as an interval by health_of
  if (is_platform_reply_id(msg.id)) { platform_ring_push(msg, now); return; }
  decode_into_cache(msg);
}

void can_buses_init() {
  can_bb.begin();
  can_bb.setBaudRate(CAN_BITRATE);
  can_bb.setMaxMB(16);
  can_bb.enableFIFO();
  can_bb.enableFIFOInterrupt();
  can_bb.onReceive(on_bb_rx);

  // Partner-absent tolerance (candidate 3, "gated broadcast";
  // generalised to all three buses 2026-07-05). Every bus is brought up
  // identically; the tolerance lives in the can_*_send() presence gate below,
  // which withholds TX whenever no partner frame has arrived within
  // BUS_PARTNER_STALENESS_US. With a bus partner-less (cone disconnected, robot
  // 12V supply off) we therefore never transmit into an un-ACKed bus, so the
  // FlexCAN TEC never climbs and the bus never enters bus-off — the failure is
  // PREVENTED, not recovered from.
  // Why not the other candidates: the pinned FlexCAN_T4 exposes no one-shot-TX
  // API (candidate 1) and no bounded bus-off-recovery setter (candidate 2 would
  // need raw ESR1/ECR register work); the software gate is the least-risky option
  // that compiles cleanly against the pinned library.
  can_cone.begin();
  can_cone.setBaudRate(CAN_BITRATE);
  can_cone.setMaxMB(16);
  can_cone.enableFIFO();
  can_cone.enableFIFOInterrupt();
  can_cone.onReceive(on_cone_rx);

  can_jugglebot.begin();
  can_jugglebot.setBaudRate(CAN_BITRATE);
  can_jugglebot.setMaxMB(16);
  can_jugglebot.enableFIFO();
  can_jugglebot.enableFIFOInterrupt();
  can_jugglebot.onReceive(on_jugglebot_rx);
}

// Per-tick RX drain budget. FlexCAN_T4::events() pops EXACTLY ONE frame from the
// peripheral's RX_SIZE_256 rxBuffer per call (FlexCAN_T4.tpp: a single
// `if (rxBuffer.size()) { pop_front; mbCallbacks; }`, not a loop), so one call per
// 1 kHz task_can_rx tick caps RX decode at ~1,000 fps. CAN3 receives ~2,240 fps of
// ODrive telemetry steady (~2,740 with a throw) — the ~5,340 fps bus total minus our
// own ~3,100 fps setpoint + time-sync TX, which the peripheral's SRX_DIS self-reception
// disable never feeds back to RX. At a 1,000 fps drain the 256-deep rxBuffer saturates
// within ~50 ms and then overwrites oldest-first: ~55% of telemetry is dropped before
// decode and the survivors reach the cache ~110 ms stale (buffer depth / arrival rate).
// We instead drain each bus to empty, capped at this budget per tick. 32 → 32,000 fps
// drain capacity, ~4x the ~7,700 fps physical 1 Mbps classical-CAN frame ceiling, so
// even a saturated/babbling bus cannot grow the buffer (steady state empties in ~3
// iterations; a full buffer clears in ~8 ticks). The cap also bounds the worst-case
// per-tick decode cost during a backlog burst.
//
// Why this keeps the 500 Hz interp ISR un-starved: decode runs in this priority-5
// FreeRTOS task, BELOW the leg-setpoint IntervalTimer ISR (leg_interp.cpp, NVIC
// priority 32, above the FreeRTOS syscall ceiling), which preempts the task at any
// instruction. A larger per-tick drain therefore cannot delay the leg setpoints; it
// only shortens the tick slice left to LOWER-priority tasks (net, telem, fault), and
// only while a backlog actually exists — the loop breaks the instant the buffer empties.
// The bounded drain also shrinks ONE specific rxBuffer race: the library's
// overwrite-oldest push advances `head` (which the task-side pop also moves) only when
// the buffer is FULL — kept near-empty, that head-vs-pop collision effectively never
// fires, where today the always-full buffer hits it on every push. (The separate
// non-atomic `_available` increment/decrement between the RX ISR and the unmasked pop
// is a pre-existing FlexCAN_T4 SPSC property, unchanged here — it self-corrects to a
// transient ±1 miscount and does not tear frame DATA while head/tail stay far apart.)
static constexpr uint8_t CAN_RX_DRAIN_BUDGET = 32;

// Drain the FlexCAN ESR1-change history (≤16 deep, captured by the library ISR on
// every error/bus-state change — CTRL_ERR_MSK is enabled) into the per-bus health
// counters. Polled every service tick to keep the history drained; error() returns
// false once it is empty (cheap — a size check, no IRQ mask), so the steady-state cost
// is one read per bus per tick. Caveat: the library drops the NEWEST snapshot when its
// 16-deep history is already full (FlexCAN_T4.tpp), so a sustained error storm that
// coincides with a multi-tick task_can_rx stall can lose transitions — the captured
// fields are best-effort under that combination. A SUSTAINED bus-off keeps re-asserting,
// so it is still captured (and fault_conf is sticky) once the storm/stall clears.
// Marginal-CAN3 diagnostic: ring of the most recent INTERESTING raw ESR1 snapshot
// words per bus (wire-error bits set, or a warning/bus-off interrupt crossing), so
// the 1 Hz print shows exactly the words that matter. Benign IDLE/RX/TX phase-flip
// snapshots (the library captures on ANY masked ESR1 change — see FlexCAN_T4.tpp
// ISR change-detect) are counted in err_events but NOT ring-recorded: at ~200
// flips/s with the 100 Hz 0x7DD active they would flush a real error word out of
// the 8-deep ring within ~40 ms — long before the 1 Hz print could show it.
// Single-writer (task_can_rx), torn reads harmless for 1 Hz debug output.
struct Esr1Ring { volatile uint32_t v[8]; volatile uint32_t n; };
static Esr1Ring s_bb_esr1{}, s_cone_esr1{}, s_jugglebot_esr1{};

template <typename BusT>
static inline void poll_bus_errors(BusT& bus, volatile BusRxHealth& h, Esr1Ring& ring) {
  CAN_error_t e;
  while (bus.error(e, /*printDetails=*/false)) {
    h.err_events++;
    const bool wire = e.ACK_ERR || e.CRC_ERR || e.FRM_ERR || e.STF_ERR ||
                      e.BIT0_ERR || e.BIT1_ERR;
    // Ring criterion: wire error, or a one-shot warning/bus-off interrupt flag
    // (TWRN_INT bit 17 / RWRN_INT bit 16 / BOFFINT bit 2 — crossings, not the
    // level bits 8/9, which stay latched while a counter sits elevated and would
    // re-flood the ring with benign flips).
    if (wire || (e.ESR1 & ((1u << 17) | (1u << 16) | (1u << 2)))) {
      ring.v[ring.n % 8u] = e.ESR1;
      ring.n = ring.n + 1u;
    }
    if (wire) {
      h.wire_errs++;
      // TX-vs-RX context at capture time (ESR1.TX bit 6 / ESR1.RX bit 3) —
      // wire-error snapshots only, so the split attributes errors, not traffic.
      if (e.ESR1 & (1u << 6)) h.err_tx_ctx++;
      if (e.ESR1 & (1u << 3)) h.err_rx_ctx++;
    }
    uint8_t f = h.err_flags;
    if (e.ACK_ERR)  { f |= BusErrFlag::ACK;     h.ack_cnt++; }
    if (e.CRC_ERR)  { f |= BusErrFlag::CRC;     h.crc_cnt++; }
    if (e.FRM_ERR)  { f |= BusErrFlag::FORM;    h.form_cnt++; }
    if (e.STF_ERR)  { f |= BusErrFlag::STUFF;   h.stuff_cnt++; }
    if (e.BIT0_ERR) { f |= BusErrFlag::BITERR0; h.bit0_cnt++; }
    if (e.BIT1_ERR) { f |= BusErrFlag::BITERR1; h.bit1_cnt++; }
    h.err_flags = f;
    if (e.RX_ERR_COUNTER > h.rec_max) h.rec_max = e.RX_ERR_COUNTER;
    // TEC, not REC, is the bus-off precursor when a bus partner is DISCONNECTED: our
    // un-ACKed TX (the 0x7DD broadcast on bb/cone/jugglebot, plus CAN3 setpoints) climbs
    // the TX error counter while REC stays 0. Track it so a "BB/Jugglebot unplugged"
    // bus-off shows a rising tec_max precursor instead of a silent jump to fault_conf=2.
    if (e.TX_ERR_COUNTER > h.tec_max) h.tec_max = e.TX_ERR_COUNTER;
    // Fault-confinement from ESR1 FLTCONF (bits 5:4): 0 active, 1 passive, >=2 bus-off.
    // Derive it from ESR1 directly — the library's FLT_CONF string compare is buggy
    // (it tests `(ESR1 & 0x30) == 0x1`, which the 0x30 mask can never satisfy, so it
    // mislabels passive as bus-off).
    const uint8_t fc = (uint8_t)((e.ESR1 >> 4) & 0x3);
    const uint8_t fc2 = (fc >= 2) ? 2 : fc;
    if (fc2 > h.fault_conf) h.fault_conf = fc2;
  }
}

// Service one bus for a tick: record the rxBuffer pressure (occupancy BEFORE
// draining = the backlog that accumulated since last tick), drain to empty capped at
// CAN_RX_DRAIN_BUDGET, flag a budget-bound (the overflow precursor), and fold in any
// bus errors. All work stays in the priority-5 CAN-RX task — see can_buses_service.
template <typename BusT>
static inline void service_bus(BusT& bus, volatile BusRxHealth& h, uint32_t can_base,
                               Esr1Ring& ring) {
  const uint16_t pre = (uint16_t)bus.getRXQueueCount();
  if (pre > h.depth_hwm) h.depth_hwm = pre;
  uint8_t n = 0;
  uint64_t r;
  // events() pops one rx frame (if any) + services tx; loop while rx frames remain.
  // The do-while always runs once, so tx is serviced every tick even when rx is idle.
  do { r = bus.events(); } while (++n < CAN_RX_DRAIN_BUDGET && (r >> 12) != 0);
  if ((r >> 12) != 0) h.cap_hits++;        // budget bound with frames still queued
  poll_bus_errors(bus, h, ring);
  // Live CAN-bus state from ONE ESR1 read (base+0x20; FlexCAN_T4 exposes no getter,
  // and CAN3's FLEXCAN3_* macros are broken in the core's imxrt.h, so use the
  // peripheral base address passed in). Neither field is sticky — both recover with
  // the bus:
  //   SYNCH (bit 18)     — 1 = controller locked onto the bus this tick (the CURRENT
  //                        "is this bus electrically alive" state).
  //   FLTCONF (bits 5:4) — live fault confinement, clamped 0 active / 1 passive /
  //                        2 bus-off (same clamp as the sticky snapshot decode in
  //                        poll_bus_errors). Feeds classify_bus_health() — the
  //                        health_of() bus-off wiring (2026-07-05).
  const uint32_t esr1 = *(volatile uint32_t*)(can_base + 0x20u);
  h.synced = (uint8_t)((esr1 >> 18) & 1u);
  const uint8_t fc_live = (uint8_t)((esr1 >> 4) & 0x3u);
  h.flt_live = (fc_live >= 2) ? 2 : fc_live;
  // Live error counters (ECR, base+0x1C): TXERRCNT [7:0], RXERRCNT [15:8]. Sampled
  // every tick; the positive inter-tick deltas accumulate into *_inc_sum so the 1 Hz
  // print can attribute error pressure to TX vs RX even when the live values decay
  // between samples (TEC/REC fall -1 per clean frame, so a 1 Hz read alone would
  // under-report bursts; a 1 kHz delta sum barely misses anything).
  const uint32_t ecr = *(volatile uint32_t*)(can_base + 0x1Cu);
  const uint8_t tec = (uint8_t)(ecr & 0xFFu);
  const uint8_t rec = (uint8_t)((ecr >> 8) & 0xFFu);
  if (tec > h.tec_live) h.tec_inc_sum += (uint32_t)(tec - h.tec_live);
  if (rec > h.rec_live) h.rec_inc_sum += (uint32_t)(rec - h.rec_live);
  h.tec_live = tec;
  h.rec_live = rec;
}

void can_buses_service() {
  service_bus(can_bb,        s_bb_rxh,        IMXRT_FLEXCAN1_ADDRESS, s_bb_esr1);
  service_bus(can_cone,      s_cone_rxh,      IMXRT_FLEXCAN2_ADDRESS, s_cone_esr1);
  service_bus(can_jugglebot, s_jugglebot_rxh, IMXRT_FLEXCAN3_ADDRESS, s_jugglebot_esr1);
}

// Drain-and-print helper for the raw-ESR1 snapshot rings (1 Hz diag). Prints only
// when new snapshots arrived since the last call; shows up to the 8 most recent.
static void print_esr1_ring(const char* name, Esr1Ring& ring, uint32_t& last_n) {
  const uint32_t n = ring.n;
  if (n == last_n) return;
  const uint32_t fresh = n - last_n;
  const uint32_t show = (fresh > 8u) ? 8u : fresh;
  Serial.printf("[canesr1]  %-9s +%lu:", name, (unsigned long)fresh);
  for (uint32_t i = 0; i < show; ++i)
    Serial.printf(" %08lx", (unsigned long)ring.v[(n - show + i) % 8u]);
  Serial.println();
  last_n = n;
}

void can_buses_print_esr1() {
  static uint32_t bb_n = 0, cone_n = 0, jb_n = 0;
  print_esr1_ring("jugglebot", s_jugglebot_esr1, jb_n);
  print_esr1_ring("bb",        s_bb_esr1,        bb_n);
  print_esr1_ring("cone",      s_cone_esr1,      cone_n);
}

// Each FlexCAN template instance is a distinct type, so a small overload per bus
// rather than a template (mirrors the original two-bus pattern).
static bool send_on(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64>& bus,
                    const ODrive::CanFrame& f) {
  CAN_message_t m;
  m.id = f.id;
  m.len = f.len;
  m.flags.extended = 0;
  memcpy(m.buf, f.buf, 8);
  return bus.write(m) > 0;
}
static bool send_on(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_64>& bus,
                    const ODrive::CanFrame& f) {
  CAN_message_t m;
  m.id = f.id;
  m.len = f.len;
  m.flags.extended = 0;
  memcpy(m.buf, f.buf, 8);
  return bus.write(m) > 0;
}
static bool send_on(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_64>& bus,
                    const ODrive::CanFrame& f) {
  CAN_message_t m;
  m.id = f.id;
  m.len = f.len;
  m.flags.extended = 0;
  memcpy(m.buf, f.buf, 8);
  return bus.write(m) > 0;
}

// ── TX serialisation (interp-ISR-safe) ───────────────────────────────────────
// All three FlexCAN writes run inside a short IRQ-off (PRIMASK) critical section.
// can_jugglebot is written by BOTH the 500 Hz leg-setpoint IntervalTimer ISR
// (leg_interp.cpp) AND several FreeRTOS tasks (time-sync 0x7DD fan-out, fault,
// RPC). FlexCAN_T4::write() is NOT reentrant — it scans the TX mailboxes and
// stores registers, with a non-atomic txBuffer.push_back on overflow — so a
// preempting writer can clobber a half-written mailbox and corrupt or drop a leg
// setpoint. A FreeRTOS mutex cannot exclude the ISR, so we mask IRQs (the same
// idiom as time_base.h atomic_read_u64 / leg_interp.cpp interp_on_setpoint). bb
// and cone have only the single time-sync writer today, but they are guarded too
// so the "every FlexCAN write is interrupt-serialised" contract is uniform and a
// future high-priority / ISR writer can't silently race. The masked region is one
// mailbox write (a few us) — well under the 500 us interp jitter budget — and it
// also makes the s_*_tx counter increment atomic.
// TODO(bench): confirm interp_max_jitter_us / interp_deadline_misses stay within
// budget with the broadcast fan-out active.
// Bus-partner presence gate (2026-07-05, generalised from the cone-absent
// tolerance): transmit only when a partner frame has been seen on
// THIS bus within BUS_PARTNER_STALENESS_US. An un-ACKed TX on a partner-less bus
// retransmits forever, pinning TEC at the error-passive threshold (128 — observed
// verbatim on CAN1 after a Ball-Butler-absent window) and, across supply ramps,
// escalating to bus-off (the CAN3 tec=254/BUSOFF history in the 2026-07-05
// marginal-CAN3 investigation). Gating at the send choke point (not per caller)
// keeps every producer — time-sync fan-out, fault machine, RPC relays, leg
// interp — bus-presence-agnostic, one enforcement point per the contract.
// FlexCAN self-reception is disabled (SRX_DIS), so our own TX never counts as
// partner presence — only real partner frames open the gate. A refused send
// returns false (callers that care, e.g. the BB command relay, already surface
// that as an RPC error) and increments tx_gated for the 1 Hz [canhealth] line.
// Residual (accepted, same as the validated cone behaviour): frames queued in
// the ≤5 s window between last RX and gate-close still retransmit un-ACKed and
// hold TEC at 128/error-passive until the bus returns — bounded, recovers, and
// never reaches bus-off on its own.
// TODO(bench): validate cone-absent on real hardware — disconnect the cone and
// confirm CAN2 never enters bus-off while CAN1/CAN3 keep broadcasting 0x7DD.
static inline bool partner_recent(const volatile uint64_t* last_rx_us,
                                  volatile BusRxHealth& h) {
  // interval: *_last_rx_us are monotonic; atomic read (torn-load guard)
  if (bus_partner_present(atomic_read_u64(last_rx_us), micros64())) return true;
  // tx_gated is written from BOTH the interp ISR and FreeRTOS tasks (unlike the
  // task_can_rx-only BusRxHealth fields), so the read-modify-write is masked —
  // same idiom as the s_*_tx counters inside the send critical section.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  h.tx_gated++;
  __set_PRIMASK(pm);
  return false;
}

bool can_bb_send(const ODrive::CanFrame& f) {
  if (!partner_recent(&s_bb_last_rx_us, s_bb_rxh)) return false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool ok = send_on(can_bb, f);
  if (ok) s_bb_tx++;
  __set_PRIMASK(pm);
  return ok;
}

bool can_cone_send(const ODrive::CanFrame& f) {
  if (!partner_recent(&s_cone_last_rx_us, s_cone_rxh)) return false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool ok = send_on(can_cone, f);
  if (ok) s_cone_tx++;
  __set_PRIMASK(pm);
  return ok;
}

bool can_jugglebot_send(const ODrive::CanFrame& f) {
  if (!partner_recent(&s_jugglebot_last_rx_us, s_jugglebot_rxh)) return false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool ok = send_on(can_jugglebot, f);
  if (ok) s_jugglebot_tx++;
  __set_PRIMASK(pm);
  return ok;
}

// Shared CAN3 command gate (declared in can_buses.h; consumed by rpc.cpp leg
// frames + platform_relay reads/writes). WARN/BUS_OFF → refuse; OK/UNKNOWN allow.
// (jugglebot_health is the health_of() classification below: RX staleness + live
// fault confinement since the 2026-07-05 bus-off wiring.)
bool jugglebot_commands_allowed() {
  const uint8_t h = can_buses_stats().jugglebot_health;
  return h != JbUdp::BusHealth::WARN && h != JbUdp::BusHealth::BUS_OFF;
}

// The bus-transmittable gate for the operator recovery one-shots
// (CLEAR_ERRORS / REBOOT_ODRIVES). Reads the LIVE ESR1.SYNCH bit maintained every
// service tick (s_jugglebot_rxh.synced, set in service_bus above). A single volatile
// byte → atomic on Cortex-M7, so no snapshot/seqlock is needed. Distinct from
// jugglebot_commands_allowed() (RX staleness + live confinement) on purpose — see
// can_buses.h for the just-repowered-bus rationale, which the 2026-07-05 bus-off
// wiring strengthens: a just-repowered bus can hold a frozen TEC=128 (closing-window
// pin) → live passive → WARN until the first successful TX decays it, and the
// recovery one-shots must reach the bus through exactly that state. The register
// read itself is the same ESR1 access already validated for fault_conf/tec_max.
bool jugglebot_bus_transmittable() {
  return s_jugglebot_rxh.synced != 0;
}

// Per-bus health for the uplink + the WARN/BUS_OFF command gates. The pure
// classifier (RX staleness + live FLTCONF) lives header-inline in can_buses.h —
// classify_bus_health, natively pinned; this wrapper feeds it the bus's own
// clock/register samples. Bus-off wiring landed 2026-07-05, closing the
// TODO(bench) that shipped with the staleness-only classifier.
static uint8_t health_of(uint64_t last_rx_us, uint8_t flt_live) {
  return classify_bus_health(last_rx_us, micros64(), flt_live);   // interval: *_last_rx_us are mono
}

CanStats can_buses_stats() {
  CanStats s;
  s.bb_rx = s_bb_rx; s.bb_tx = s_bb_tx;
  s.cone_rx = s_cone_rx; s.cone_tx = s_cone_tx;
  s.jugglebot_rx = s_jugglebot_rx; s.jugglebot_tx = s_jugglebot_tx;
  // 64-bit timestamps: read atomically (written atomically in the RX callbacks).
  // can_buses_stats runs in lower-priority tasks than the CAN-RX writer, so a
  // plain two-word load could tear across a writer preemption (esp. at the ~71 min
  // micros64 high-word wrap) and mis-classify bus health.
  // flt_live: volatile uint8_t, single writer (task_can_rx) — a byte load is
  // atomic on Cortex-M7, so no snapshot/seqlock is needed. Per-bus inputs only:
  // each bus's health is independent of the other two by construction.
  s.bb_health = health_of(atomic_read_u64(&s_bb_last_rx_us), s_bb_rxh.flt_live);
  s.cone_health = health_of(atomic_read_u64(&s_cone_last_rx_us), s_cone_rxh.flt_live);
  s.jugglebot_health = health_of(atomic_read_u64(&s_jugglebot_last_rx_us), s_jugglebot_rxh.flt_live);
  return s;
}

// Field-by-field copy of a volatile health block. Each field is a single word, so
// each load is atomic; a torn read ACROSS fields is harmless for 1 Hz debug output.
static BusRxHealth snapshot_bus(const volatile BusRxHealth& h) {
  BusRxHealth o;
  o.depth_hwm  = h.depth_hwm;
  o.cap_hits   = h.cap_hits;
  o.err_events = h.err_events;
  o.wire_errs  = h.wire_errs;
  o.err_flags  = h.err_flags;
  o.rec_max    = h.rec_max;
  o.tec_max    = h.tec_max;
  o.fault_conf = h.fault_conf;
  o.synced     = h.synced;
  o.ack_cnt    = h.ack_cnt;
  o.crc_cnt    = h.crc_cnt;
  o.form_cnt   = h.form_cnt;
  o.stuff_cnt  = h.stuff_cnt;
  o.bit0_cnt   = h.bit0_cnt;
  o.bit1_cnt   = h.bit1_cnt;
  o.err_tx_ctx = h.err_tx_ctx;
  o.err_rx_ctx = h.err_rx_ctx;
  o.tec_live   = h.tec_live;
  o.rec_live   = h.rec_live;
  o.flt_live   = h.flt_live;
  o.tec_inc_sum = h.tec_inc_sum;
  o.rec_inc_sum = h.rec_inc_sum;
  o.tx_gated    = h.tx_gated;
  return o;
}

CanRxHealth can_buses_rx_health() {
  CanRxHealth s;
  s.bb        = snapshot_bus(s_bb_rxh);
  s.cone      = snapshot_bus(s_cone_rxh);
  s.jugglebot = snapshot_bus(s_jugglebot_rxh);
  s.decode_short    = s_decode_short;
  s.decode_bad_axis = s_decode_bad_axis;
  return s;
}

// ── Marginal-CAN3 diagnostic: decoded bit-timing register dump ────────────────
// Reads CTRL1 (base+0x4) per bus and CCM_CSCMR2 (CAN root clock) and prints the
// ACTUAL timing the silicon runs: prescaler, segment lengths, resulting bit rate
// and sample point. FlexCAN_T4::setBaudRate picks these from an internal table;
// this dump is the ground truth for comparing against the ODrive Pro / Platform
// Teensy timing when hunting the marginal-CAN3 error source.
static void dump_one_timing(const char* name, uint32_t can_base, uint32_t can_clk_hz) {
  const uint32_t ctrl1 = *(volatile uint32_t*)(can_base + 0x4u);
  const uint32_t presdiv = (ctrl1 >> 24) & 0xFFu;
  const uint32_t rjw     = (ctrl1 >> 22) & 0x3u;
  const uint32_t pseg1   = (ctrl1 >> 19) & 0x7u;
  const uint32_t pseg2   = (ctrl1 >> 16) & 0x7u;
  const uint32_t propseg = ctrl1 & 0x7u;
  // Actual tq counts are field+1 (sync seg is a fixed 1 tq).
  const uint32_t ntq = 1u + (propseg + 1u) + (pseg1 + 1u) + (pseg2 + 1u);
  const uint32_t sclk = can_clk_hz / (presdiv + 1u);
  const uint32_t rate = sclk / ntq;
  const uint32_t sp_x10 = (1u + (propseg + 1u) + (pseg1 + 1u)) * 1000u / ntq;
  Serial.printf("[cantiming] %-9s CTRL1=0x%08lx presdiv=%lu propseg=%lu pseg1=%lu pseg2=%lu rjw=%lu"
                " ntq=%lu rate=%lu sp=%lu.%lu%% sjw=%lutq\n",
                name, (unsigned long)ctrl1, (unsigned long)presdiv, (unsigned long)propseg,
                (unsigned long)pseg1, (unsigned long)pseg2, (unsigned long)rjw,
                (unsigned long)ntq, (unsigned long)rate,
                (unsigned long)(sp_x10 / 10u), (unsigned long)(sp_x10 % 10u),
                (unsigned long)(rjw + 1u));
}

void can_buses_dump_timing() {
  // CAN root clock from CCM_CSCMR2: CAN_CLK_SEL [9:8] (0=pll3/8=60M, 1=osc 24M,
  // 2=pll3/6=80M), CAN_CLK_PODF [7:2] divides by (podf+1).
  const uint32_t cscmr2 = CCM_CSCMR2;
  const uint32_t sel  = (cscmr2 >> 8) & 0x3u;
  const uint32_t podf = (cscmr2 >> 2) & 0x3Fu;
  uint32_t root = 0;
  if (sel == 0) root = 60000000u;
  else if (sel == 1) root = 24000000u;
  else if (sel == 2) root = 80000000u;
  const uint32_t can_clk = root ? root / (podf + 1u) : 0;
  Serial.printf("[cantiming] CCM_CSCMR2=0x%08lx clk_sel=%lu podf=%lu can_clk=%luHz\n",
                (unsigned long)cscmr2, (unsigned long)sel, (unsigned long)podf,
                (unsigned long)can_clk);
  if (!can_clk) { Serial.println("[cantiming] CAN clock disabled?!"); return; }
  dump_one_timing("bb",        IMXRT_FLEXCAN1_ADDRESS, can_clk);
  dump_one_timing("cone",      IMXRT_FLEXCAN2_ADDRESS, can_clk);
  dump_one_timing("jugglebot", IMXRT_FLEXCAN3_ADDRESS, can_clk);
}

}  // namespace CanBridge
