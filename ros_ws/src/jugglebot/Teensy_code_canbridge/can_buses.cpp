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
#include "gpio_poll.h"       // hand ball-sensor TxSdo replies (gpio_poll_record)

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
// OPERATING CONFIG since 2026-07-31: the jugglebot role runs on the CAN2
// controller and the cone role on CAN3 (physical plugs swapped to match). The
// bridge's CAN3 analog drive path developed a LOAD-DEPENDENT fault — it drives
// the 1-node cone bus cleanly at 100 Hz but fails within seconds against the
// 8-node Jugglebot chain (bit1-dominant TX corruption → error-passive/BUS_OFF;
// logbook 2026-07-31-can3-drive-path-fault-jugglebot-to-can2). Jugglebot
// therefore lives on CAN2 until the CAN3 transceiver/stub is repaired; the
// cone's light load tolerates the weak path. All bus config is per-instance
// (same CAN_BITRATE, same init), so each role carries its full behaviour to
// its controller; the ESR1 base addresses in service_bus() are swapped to
// match. Wire-slot names (bus1_health, can1_* profile slot, can3_errors row)
// are ROLE-keyed and unchanged. Revert both declarations + the ESR1 addresses
// when the CAN3 path is fixed and re-validated.
static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_64> can_cone;
static FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_64> can_jugglebot;

static volatile uint32_t s_bb_rx = 0, s_bb_tx = 0, s_cone_rx = 0, s_cone_tx = 0,
                         s_jugglebot_rx = 0, s_jugglebot_tx = 0;
static volatile uint64_t s_bb_last_rx_us = 0, s_cone_last_rx_us = 0, s_jugglebot_last_rx_us = 0;

// ── Cone-bus ROLE stamps (2026-08-16, clapboard-can3-integration) ────────────
// s_cone_last_rx_us above is the SHARED, ID-AGNOSTIC stamp and is the sole input
// to the TX presence gate (partner_recent → can_cone_send). These two are for
// HEALTH REPORTING ONLY and are stamped behind the is_cone_id / is_clapboard_id
// discriminators, so cone_health can answer "is a CATCHING CONE attached" rather
// than "is anything attached" — the lie the clapboard would otherwise make it
// tell, since the two devices are mutually exclusive on this one segment.
// NEVER move the gate stamp behind a discriminator: it would close the gate for
// whichever device is not the one the branch names, stopping the 100 Hz 0x7DD
// broadcast, so the clapboard never anchors its wall clock, never emits a fire
// event and never leaves its screensaver — with no error anywhere. Same writer
// (on_cone_rx, task_can_rx) and same 64-bit atomic discipline as the stamp above.
static volatile uint64_t s_cone_only_last_rx_us = 0;   // 0x7E0 / 0x7E1 only
static volatile uint64_t s_clap_last_rx_us = 0;        // 0x7E8-0x7EF only

// RX-health observability counters (see can_buses.h "RX-health observability").
// Single-writer: task_can_rx — the service loop (depth/cap/bus-error fields) and the
// decode callback (decode_* fields, and s_enc_frames below, which is in exactly that
// class). EXCEPTION: tx_gated is written by the TX gate in
// partner_recent() from sender contexts (interp ISR + tasks) under a PRIMASK-masked
// increment. (One boot-window exception: before the first
// events() call flips the library's isEventsUsed, the FlexCAN ISR dispatches the decode
// callback directly, so the decode_* increments briefly run in ISR context — harmless,
// since task_diag is not reading yet.) Each field is one word (atomic load/store on
// Cortex-M7) and cumulative/sticky; err_flags is a producer-local read-or-store. The
// cross-task read in task_diag therefore needs no lock and no reset drops a transient.
static volatile BusRxHealth s_bb_rxh{}, s_cone_rxh{}, s_jugglebot_rxh{};
static volatile uint32_t s_decode_short = 0, s_decode_bad_axis = 0;
// Per-axis get_encoder_estimate frames decoded AND cached — the same
// single-writer class as the two decode_* counters above (task_can_rx, one word,
// cumulative, never reset), so it inherits their lock-free cross-task read.
// Rationale for a PER-AXIS counter where the aggregates already exist: an
// aggregate cannot see one axis of seven stop broadcasting, and that is exactly
// the shape the 2026-08-12 S1 bag forensics found. See CanRxHealth::enc_frames.
static volatile uint32_t s_enc_frames[NUM_AXES] = {0};

// ── RX-ring TRUE-occupancy probe (FW 13; contract in can_buses.h) ───────────
// Written ONLY by service_bus (task_can_rx, priority 5); read by task_telem
// (priority 3) under PRIMASK. The reader masks and the writer deliberately does
// NOT: the writer OUTRANKS the reader, so the reader can never preempt a
// half-written record, and masking in the writer would put IRQ-off time on a
// path that runs ~2,240 times a second beneath a 500 Hz hard-deadline ISR — the
// opposite of "instrumentation only". (gpio_poll.h documents the same priority
// argument in the other direction, where its writer is the LOWER-priority task
// and therefore must mask.) The masked whole-record read matters because
// `depth` and `avail` are only meaningful AS A PAIR: catching one from tick N
// and the other from tick N+1 would fabricate or erase a leak.
static BusRingProbe s_bb_ringprobe{}, s_cone_ringprobe{}, s_jugglebot_ringprobe{};
static uint32_t s_ring_probe_ticks = 0;

// ── Jugglebot delivery-lag accumulator (FW 13; contract in can_buses.h) ─────
// Same writer/reader classes and the same masking argument as the ring probe
// above: written in on_jugglebot_rx (task_can_rx), read by task_telem at 1 Hz.
//
// The gap bound that decides when the 16-bit capture-stamp unwrap is still
// trustworthy. The jugglebot bus carries ~2,240 frames/s steady, i.e. ~0.45 ms
// between deliveries, against a 65.536 ms wrap — so 50 ms is ~110 normal
// inter-frame gaps and still 15 ms clear of the wrap. Exceeding it means the bus
// genuinely went quiet (ODrives unpowered) or task_can_rx stalled for 50 ms, and
// in EITHER case the number of elapsed wraps is unknowable. Measured on the
// DECODE side because the capture side is the very quantity in doubt; the two
// differ by the delivery lag, which is the ~100 ms the whole investigation is
// chasing, and 15 ms of headroom plus the reseed's own visibility is the honest
// trade. Reseeding beats accumulating a wrong unwrap: a corrupted arrival clock
// poisons every later sample silently and forever, a reseed costs one comparable
// pair and shows up in lag_reseeds.
static constexpr uint64_t JB_LAG_UNWRAP_GAP_US = 50000ull;
// |lag| beyond this is unwrap corruption, not physics: the leak's delivery lag
// is capped at one ring lap (256 slots ≈ 135 ms at jb rates). See the
// PHYSICAL-CAP guard in jb_lag_fold.
static constexpr uint64_t JB_LAG_PHYS_CAP_US   = 200000ull;
static uint16_t s_jb_ts_prev      = 0;   // previous frame's FlexCAN capture stamp
static uint64_t s_jb_arrival_us   = 0;   // unwrapped capture clock since the seed
static uint64_t s_jb_anchor_us    = 0;   // micros64() at the seed
static uint64_t s_jb_dec_prev_us  = 0;   // micros64() at the previous delivery (gap test)
static int32_t  s_jb_lag_now_us   = 0;   // (decode elapsed) - (capture elapsed) since the seed
static int32_t  s_jb_lag_hwm_us   = 0;
static uint32_t s_jb_lag_frames   = 0;   // frames folded since boot (differenced per window)
static uint32_t s_jb_lag_reseeds  = 0;
static bool     s_jb_lag_seeded   = false;
static bool     s_jb_lag_reseeded_since_read = false;

// Fold one delivered jugglebot frame into the arrival clock. Called from
// on_jugglebot_rx for EVERY frame — before any id/type classification, so the
// clock measures DELIVERY and never a subset of it. A handful of integer ops and
// no division, no masking, no branch on bus state.
static inline void jb_lag_fold(uint16_t ts, uint64_t now_mono) {
  if (!s_jb_lag_seeded) {
    s_jb_lag_seeded  = true;
    s_jb_arrival_us  = 0;
    s_jb_anchor_us   = now_mono;
    s_jb_lag_now_us  = 0;
    s_jb_lag_hwm_us  = 0;
  } else if ((now_mono - s_jb_dec_prev_us) >= JB_LAG_UNWRAP_GAP_US) {
    // Unwrap no longer trustworthy — reseed and say so, rather than adding an
    // unknown multiple of 65.536 ms to the clock and reporting it as lag.
    ++s_jb_lag_reseeds;
    s_jb_lag_reseeded_since_read = true;
    s_jb_arrival_us  = 0;
    s_jb_anchor_us   = now_mono;
    s_jb_lag_now_us  = 0;
    s_jb_lag_hwm_us  = 0;   // the hwm is only meaningful against ONE reference
  } else {
    s_jb_arrival_us += (uint64_t)(uint16_t)(ts - s_jb_ts_prev);
    // Telescopes exactly to (delivery lag now) - (delivery lag at the seed): the
    // capture stamps are hardware, taken at reception, so everything the ring
    // does to the frame afterwards lands in this difference and nowhere else.
    const int64_t lag = (int64_t)(now_mono - s_jb_anchor_us) - (int64_t)s_jb_arrival_us;
    // PHYSICAL-CAP guard: one blind spot survives the decode-gap reseed — a bus
    // quiet of ~66-82 ms behind a still-draining backlog keeps DECODE gaps under
    // the reseed line while the CAPTURE clock wraps once, stepping the unwrap by
    // -65.536 ms permanently and silently. But the leak's lag is physically
    // bounded by one ring lap (~135 ms), so any |lag| beyond ~200 ms is unwrap
    // corruption, not measurement: force the reseed and count it, so the series
    // segments visibly instead of carrying a wrapped reference forever.
    if (lag > (int64_t)JB_LAG_PHYS_CAP_US || lag < -(int64_t)JB_LAG_PHYS_CAP_US) {
      ++s_jb_lag_reseeds;
      s_jb_lag_reseeded_since_read = true;
      s_jb_arrival_us  = 0;
      s_jb_anchor_us   = now_mono;
      s_jb_lag_now_us  = 0;
      s_jb_lag_hwm_us  = 0;
    } else {
      const int32_t lag32 = (int32_t)lag;
      s_jb_lag_now_us = lag32;
      if (lag32 > s_jb_lag_hwm_us) s_jb_lag_hwm_us = lag32;
    }
  }
  s_jb_ts_prev     = ts;
  s_jb_dec_prev_us = now_mono;
  ++s_jb_lag_frames;
}

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
      // Count AFTER the cache write, deliberately: the useful invariant is
      // "enc_frames[axis] advanced ⇒ write_pos_vel completed for that axis", so a
      // bag showing a stalled VALUE against an advancing counter places the fault
      // at or above this line (a stale estimate on the wire), never below it.
      // Counting before the write would only have proved the frame was classified.
      s_enc_frames[axis]++;
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
    case ODriveCmd::TxSdo:
      // The bridge's ONLY TxSdo consumer: the hand ball-sensor poll's
      // get_gpio_states reply (gpio_poll.cpp). No encoder-search handler has ever
      // existed here. The endpoint match is what stops a reply to some other SDO
      // read from masquerading as a sensor sample; the value is a GPIO bitmask, so
      // it decodes as uint32 (a float32 unpack would reinterpret the bits).
      if (axis == HAND_AXIS) {
        const ODrive::SdoResponseU32 r = ODrive::decode_sdo_response_u32(d);
        if (r.endpoint_id == EndpointId::odrive_pro_0_6_11::get_gpio_states)
          // Stamped HERE, at arrival, not when task_homing drains the mailbox:
          // t_bridge_us is contractually "at the last good TxSdo reply", and the
          // drain is up to a task tick later.
          gpio_poll_record(r.value, now_wall_us(), micros64());
      }
      break;
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

// CAN2 cone-ROLE bus: no ODrive here → count, then copy the frame into the
// cone-uplink ring for the Jetson relay. The RX timestamp also drives the
// bus-partner presence gate in can_cone_send() (cone-absent tolerance).
//
// The segment carries EXACTLY ONE peripheral, but which one is a physical choice:
// the catching cone (0x7E0/0x7E1) or the electronic clapboard (0x7E8-0x7EF). Both
// the counter and the ring push stay ID-AGNOSTIC — the relay forwards every frame
// verbatim, which is the whole reason the clapboard uplink needed no wire change.
// Only the two HEALTH stamps discriminate.
static void on_cone_rx(const CAN_message_t& msg) {
  // DUAL-USE split: s_cone_last_rx_us drives the cone-presence gate + health
  // (INTERVAL → micros64); the ring t_bridge_us is a wire-bound absolute timestamp (→ wall).
  const uint64_t now      = now_wall_us();   // wire-bound absolute timestamp — wall by contract
  const uint64_t now_mono = micros64();      // interval clock for the gate + health stamp
  s_cone_rx++;
  // UNCONDITIONAL, and it must stay that way — the TX presence gate reads this and
  // only cares that SOMEONE is on the bus to ACK. Pinned by
  // tests/firmware/test_cone_rx_role_lint.py; rationale in can_buses.h.
  atomic_write_u64(&s_cone_last_rx_us, now_mono);   // 64-bit monotonic; read as an interval by the cone gate + health_of
  // Role discrimination — HEALTH ONLY (see the stamp declarations above). Frames
  // in the unallocated 0x7E2-0x7E7 gap, and the 0x7DD broadcast if it were ever
  // looped back, match neither and stamp neither: they are evidence that the bus
  // is alive (already recorded above), not evidence of WHICH device is attached.
  if (is_clapboard_id(msg.id)) {
    atomic_write_u64(&s_clap_last_rx_us, now_mono);
  } else if (is_cone_id(msg.id)) {
    atomic_write_u64(&s_cone_only_last_rx_us, now_mono);
  }
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
  // FW 13 RING_DIAG: fold the HARDWARE capture stamp into the delivery-lag
  // accumulator. Placed here, ABOVE the platform-reply branch, so every
  // delivered frame counts: the quantity is how late the RING hands frames over,
  // which has nothing to do with what any given frame turns out to contain.
  jb_lag_fold(msg.timestamp, now_mono);
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

  // ── setMaxMB(24) — 16 TX mailboxes on the Jugglebot bus (2026-08-09) ───────
  // JUGGLEBOT BUS ONLY. FW 10's ERR_TIMEOUT fix: with the old setMaxMB(16) this
  // bus had EIGHT TX mailboxes, and the 500 Hz interp ISR fills six of them in
  // one back-to-back burst (leg_interp.cpp:533-541). A hand dispatch landing in
  // the post-burst window found ≤2 mailboxes free, so FlexCAN_T4::write()
  // returned -1 (deferral into the software ring) and hand_ops reported
  // ERR_TIMEOUT. Measured on the bench 2026-08-09: 0/40 failures with the leg
  // stream idle, 15/40 with it running (Fisher p = 8.5e-09), and the per-stage
  // split pre1 0 / pre2 7 / traj 8 reads out the pending count directly.
  //
  //  * 24 ⇒ 16 TX mailboxes. mailboxOffset() is 6 + (RFFN+1)*2 and is FIXED at 8
  //    (6 MBs for the RX-FIFO engine + 2 for the 8-entry ID filter table) —
  //    INDEPENDENT of MAXMB (FlexCAN_T4.tpp:439-444; RFFN is 0 because nothing
  //    in this tree ever calls setRFFN/setFIFOFilter and begin() only ORs into
  //    CTRL2). So TX mailboxes = MAXMB − 8: 16 → 8, 24 → 16.
  //  * SIZING: the DESIGN BOUND on pending is 8 — the 6-frame leg burst plus the
  //    only two other producers on this bus (the 100 Hz 0x7DD broadcast and the
  //    50 Hz hand SDO poll). The bench MEASURED at most 7: pre1_fail = 0 across
  //    40 trials means P = 8 never occurred, i.e. the two non-leg frames were
  //    never simultaneously resident. Size to the design bound, not the measured
  //    one — 8 + a worst-case 3-frame hand dispatch = 11 ≤ 16, with margin for a
  //    future producer. 24 rather than 32/64 because the write() scan below is
  //    linear in MAXMB and runs inside our PRIMASK region.
  //  * COST: zero MCU RAM — mailboxes are FlexCAN peripheral RAM; the MCU-side
  //    buffers are the RX_SIZE_256 / TX_SIZE_64 template parameters and are
  //    untouched. The FlexCAN ISR scan does NOT grow either: flexcan_interrupt
  //    computes exit_point = 64 - __builtin_clzll(iflag|1) and breaks at the
  //    highest FLAGGED mailbox (tpp:1194-1197), so unused high MBs are never
  //    walked. What does grow is write()'s free-mailbox scan (tpp:1038), which
  //    runs inside can_jugglebot_send()'s IRQ-off region: ~1 iteration on the
  //    happy path (it breaks at the first inactive MB) and only the full 16-MB
  //    walk on the congested path — i.e. exactly where a longer scan is bought
  //    by not deferring. Sub-µs per send against the 2000 µs interp period;
  //    watch interp_max_jitter_us / interp_deadline_misses on the bench anyway.
  //    UNVERIFIED against the RM: FlexCAN's TX arbitration scans all ENABLED
  //    mailboxes each round and the RM notes the scan duration scales with MAXMB,
  //    which at a low protocol clock can cost a back-to-back transmission slot.
  //    That hazard degrades BUS TX THROUGHPUT, not ISR timing — so watch the
  //    jugglebot TX fps (Profile wire slot 1, can1_tx), not just interp jitter:
  //    interp jitter would stay clean while the bus quietly lost slots.
  //  * ORDER IS LOAD-BEARING: setMaxMB MUST precede enableFIFO() +
  //    enableFIFOInterrupt(), as it does here. setMaxMB internally does
  //    disableFIFO → write MCR[MAXMB] → enableFIFO (tpp:446-457), and
  //    enableFIFO() calls writeIMASK(0ULL) which CLEARS BUF5M, while
  //    enableFIFOInterrupt() early-returns if BUF5M is already set
  //    (tpp:182-226). Calling setMaxMB AFTER enableFIFO therefore silently kills
  //    the RX-FIFO interrupt and the bus goes RX-DARK with no error reported.
  //    setMaxMB also clears every mailbox and RXIMR mask, so it is init-only —
  //    calling it at runtime would destroy in-flight leg setpoints.
  //  * EFFECT: converts a deferral into a hardware-queued (late) transmission.
  //    The ERR_TIMEOUTs go away; the WIRE LATENCY DOES NOT — 0x0C7, 0x0CB and
  //    0x6D0 all rank below every leg id (0x00C-0x0AC), so they still go out
  //    after the burst drains. It also parks the vendored events() mb == -1
  //    refill defect (no break; writes one frame into every free mailbox while
  //    popping others — logbook 2026-08-02 addendum § A6): at the observed
  //    occupancy the software ring is never entered, so that path is
  //    unreachable. It becomes reachable again if a future TX producer doubles
  //    the burst — and the parking is ONE-DIRECTIONAL in blast radius: when the
  //    ring IS re-entered, that break-less loop now duplicates the peeked frame
  //    into up to 16 mailboxes instead of 8, so a deferred 0x6D0 duplicates twice
  //    as hard as it did on FW 9. Anything that re-opens the deferral path must
  //    FIX THE VENDORED LOOP, not just re-size the mailboxes.
  //    ✅ DISCHARGED, FW 14 (2026-08-14): the vendored loop now has its `break`
  //    (lib/FlexCAN_T4/PROVENANCE.md § P4), so a deferred frame goes into exactly ONE
  //    free mailbox. The parking argument above still holds and is still why this is a
  //    no-op today (defer == 0 on every bus, so the branch never runs) — what changed
  //    is that re-opening deferral is no longer conditional on remembering to fix it.
  //
  // NOT applied to bb/cone: both deferred 0 frames across the whole bench arm and
  // each carries only the 100 Hz 0x7DD fan-out plus event-driven RPC relays
  // (bb: rpc.cpp:137 send_bb_frame), never a 500 Hz burst — so a symmetric change
  // would be risk without need.
  can_jugglebot.begin();
  can_jugglebot.setBaudRate(CAN_BITRATE);
  can_jugglebot.setMaxMB(24);
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
// FreeRTOS task, BELOW the leg-setpoint IntervalTimer ISR (leg_interp.cpp:608, NVIC
// priority 16, above the FreeRTOS syscall ceiling), which preempts the task at any
// instruction. The number matters: the ceiling
// (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 2 << (8-4)) is itself 32, and BASEPRI
// masks any interrupt whose priority VALUE is >= BASEPRI — so an ISR at 32 would sit
// EXACTLY AT the ceiling and be masked by every RTOS critical section, contradicting the
// conclusion this comment draws. At the real value of 16 (16 < 32) both the number and
// the conclusion hold. (This line said 32 until 2026-08-09; the conclusion was always
// right, but a reader who trusted the number over the conclusion would reason correctly
// about masking and land on the opposite answer.)
// A larger per-tick drain therefore cannot delay the leg setpoints; it
// only shortens the tick slice left to LOWER-priority tasks (net, telem, fault), and
// only while a backlog actually exists — the loop breaks the instant the buffer empties.
// The bounded drain also shrinks ONE specific rxBuffer race: the library's
// overwrite-oldest push advances `head` (which the task-side pop also moves) only when
// the buffer is FULL — kept near-empty, that head-vs-pop collision effectively never
// fires, where today the always-full buffer hits it on every push. (The separate
// non-atomic `_available` increment/decrement between the RX ISR and the unmasked pop
// is a pre-existing FlexCAN_T4 SPSC property, unchanged here — it self-corrects to a
// transient ±1 miscount and does not tear frame DATA while head/tail stay far apart.)
// ⚠️ REFUTED 2026-08-14: the parenthetical above is WRONG on both counts. The race is
// ONE-directional (the ISR preempts the pop, never the reverse), so the miscount is a
// MONOTONE LEAK, not a self-correcting transient — `_available` under-counts forever,
// stranding frames and turning the ring into an uptime-ratcheting delay line; and the
// "kept near-empty" evidence above was `depth_hwm`, which is derived from `_available`
// itself (circular). Kept verbatim as the historical record; the correct account is the
// BusRingProbe contract (can_buses.h) and
// logbook/2026-08-14-ring-audit-available-leak-delay-line.md § 6. Fix sequenced as
// FW 14, after the RING_DIAG occupancy measurement convicts on a number.
// ✅ CONVICTED, THEN FIXED, FW 14 (2026-08-14). RING_DIAG on a 4.03 h board read
// leak_jb = 247-248 (true_depth vs avail_reported 0; hwm 249 ≈ 97 % of one lap) against
// 1 on bb and 0 on cone — the predicted arrival × pop ordering — with e2e leg lag
// 19.9 ms fresh vs 252.2 ms at 3.80 h. The pop now runs inside the bus's own
// NVIC_DISABLE_IRQ window (lib/FlexCAN_T4/PROVENANCE.md § P3), which closes BOTH races
// this paragraph discusses: the `_available` leak AND the ring-full head-vs-pop
// collision, so the "bounded drain shrinks it" mitigation above is no longer the only
// thing standing between us and the tear. The drain budget itself is unchanged and
// still earns its place for every other reason given above. Post-fix acceptance is
// RING_DIAG leak ≡ 0 on all three buses at any uptime — same instrument, now used to
// prove its own fix.
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
                               Esr1Ring& ring, BusRingProbe& probe) {
  const uint16_t pre = (uint16_t)bus.getRXQueueCount();
  if (pre > h.depth_hwm) h.depth_hwm = pre;
  uint8_t n = 0;
  uint64_t r;
  // events() pops one rx frame (if any) + services tx; loop while rx frames remain.
  // The do-while always runs once, so tx is serviced every tick even when rx is idle.
  do { r = bus.events(); } while (++n < CAN_RX_DRAIN_BUDGET && (r >> 12) != 0);
  if ((r >> 12) != 0) h.cap_hits++;        // budget bound with frames still queued
  // ── FW 13: TRUE post-drain occupancy — the leak, measured ─────────────────
  // THE PROBE POINT IS THE POINT. The loop above exits when `_available` reads
  // 0, so at THIS instant the reported count is zero by construction while the
  // ring's head/tail still say what is actually stranded in it. `depth - avail`
  // is therefore the leak itself, not an inference from it, and every counter
  // the bridge had before this line — depth_hwm and cap_hits right above
  // included — is computed from `_available` and cannot see it.
  // (The other exit is the budget bound, where avail is non-zero and both terms
  // are elevated; the difference is still the leak, which is why the subtraction
  // is taken rather than reading depth alone.)
  uint16_t ph, pt, pa, pd;
  bus.rxRingProbe(ph, pt, pa, pd);          // reads head/tail BEFORE `_available`,
                                            // so a racing push can only shrink the
                                            // reported leak — never invent one
  probe.depth = pd;
  probe.avail = pa;
  if (pd > probe.depth_hwm) probe.depth_hwm = pd;
  if (pd > pa) {
    const uint16_t leak = (uint16_t)(pd - pa);
    if (leak > probe.leak_hwm) probe.leak_hwm = leak;
  }
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
  // Sustained-confinement tracking for the COMMAND gate (2026-07-29 CAN3 flap;
  // the normative split lives at classify_command_gate in can_buses.h). Derived
  // from the SAME ESR1 read — no extra register access. Edge into confinement
  // stamps the entry time; leaving it clears both fields, so a bus that keeps
  // flapping in and out never accumulates toward the sustain threshold. That is
  // the intent: a flapping bus is self-healing by definition and must not gate
  // commands, while a genuinely stuck one crosses the threshold and does.
  if (h.flt_live == 0) {
    h.flt_passive_since_us = 0;
    h.flt_sustained        = 0;
  } else {
    const uint64_t now_mono = micros64();
    if (h.flt_passive_since_us == 0) h.flt_passive_since_us = now_mono;
    h.flt_sustained =
        ((now_mono - h.flt_passive_since_us) >= CAN_PASSIVE_SUSTAIN_US) ? 1u : 0u;
  }
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
  service_bus(can_bb,        s_bb_rxh,        IMXRT_FLEXCAN1_ADDRESS, s_bb_esr1,        s_bb_ringprobe);
  // Operating config (see instance declarations): cone rides FLEXCAN3,
  // jugglebot FLEXCAN2 — the ESR1 base must follow the role's controller.
  service_bus(can_cone,      s_cone_rxh,      IMXRT_FLEXCAN3_ADDRESS, s_cone_esr1,      s_cone_ringprobe);
  service_bus(can_jugglebot, s_jugglebot_rxh, IMXRT_FLEXCAN2_ADDRESS, s_jugglebot_esr1, s_jugglebot_ringprobe);
  // One increment per TICK, not per bus: it is the denominator for the extrema
  // above, and all three buses are probed exactly once each per tick.
  ++s_ring_probe_ticks;
}

CanRingProbe can_buses_ring_probe() {
  CanRingProbe s{};
  // Whole-record copy under PRIMASK — see the declaration comment: the reader
  // (task_telem, priority 3) is OUTRANKED by the writer (task_can_rx, priority
  // 5), so masking here is what stops a service tick landing between the `depth`
  // and `avail` reads and fabricating a leak that never existed.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s.bb          = s_bb_ringprobe;
  s.cone        = s_cone_ringprobe;
  s.jugglebot   = s_jugglebot_ringprobe;
  s.probe_ticks = s_ring_probe_ticks;
  __set_PRIMASK(pm);
  // The FIFO counters live in the library instances and are written ONLY by the
  // FlexCAN ISR (vendored patch P2), so they are exact and each is a single
  // aligned word — read outside the mask, deliberately: they are cumulative and
  // independent of the depth/avail pair, so there is nothing to tear across.
  s.bb.fifo_overflows        = can_bb.getFIFOOverflowCount();
  s.bb.fifo_warns            = can_bb.getFIFOWarnCount();
  s.cone.fifo_overflows      = can_cone.getFIFOOverflowCount();
  s.cone.fifo_warns          = can_cone.getFIFOWarnCount();
  s.jugglebot.fifo_overflows = can_jugglebot.getFIFOOverflowCount();
  s.jugglebot.fifo_warns     = can_jugglebot.getFIFOWarnCount();
  return s;
}

JbLagProbe can_buses_jb_lag_take() {
  JbLagProbe s{};
  // Same masking argument as can_buses_ring_probe: lag_now_us, arrival_us and
  // the frame count are read as ONE sample (the host differences arrival_us and
  // frames across windows, so a torn pair would put a frame's interval in the
  // wrong window), and the reseed flag must be consumed exactly once.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s.lag_now_us = s_jb_lag_now_us;
  s.lag_hwm_us = s_jb_lag_hwm_us;
  s.arrival_us = s_jb_arrival_us;
  s.frames     = s_jb_lag_frames;
  s.reseeds    = s_jb_lag_reseeds;
  s.seeded     = s_jb_lag_seeded;
  s.reseeded_since_read        = s_jb_lag_reseeded_since_read;
  s_jb_lag_reseeded_since_read = false;   // one window sees each reseed, exactly once
  __set_PRIMASK(pm);
  return s;
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

// ── TX-pressure instrumentation, inside the EXISTING masked region ───────────
// tx_deferred and tx_q_hwm ride the mask the mailbox write already holds: no new
// critical section, no added ISR latency. getTXQueueCount() is txBuffer.size(),
// which is a single member load (circular_buffer.h: `return _available;`) — no
// loop, no allocation, no FreeRTOS call — so the audited hard invariant at
// leg_interp.cpp:598-608 (interp_isr and everything it calls make ZERO FreeRTOS
// API calls) is preserved.
//
// The high-water mark is sampled HERE, at the send instant, and not on the 1 kHz
// service tick. A queued frame leaves in ~115 us at 1 Mbit, so by the time the
// service tick looks the queue is back to ~0: 1 kHz sampling would read zero
// through precisely the bursts that make a send defer.
//
// What the sample means differs by path, and neither reading is wrong: on the
// DEFERRAL path the frame just written is itself in the ring, so the sample
// includes it; on the mailbox path the frame went straight to hardware and is
// not in the ring at all, so the sample is the backlog this caller found already
// pending. Both answer "how deep was the software queue at this send instant",
// which is the pressure signal — not "how many frames are waiting because of me".
//
// `ok == false` is a DEFERRAL, not a drop — write() returned -1 and the frame is
// in the software txBuffer awaiting the TX-complete ISR. See can_buses.h for the
// two paths that do genuinely lose a frame.

bool can_bb_send(const ODrive::CanFrame& f) {
  if (!partner_recent(&s_bb_last_rx_us, s_bb_rxh)) return false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool ok = send_on(can_bb, f);
  if (ok) s_bb_tx++; else s_bb_rxh.tx_deferred++;
  const uint16_t q = (uint16_t)can_bb.getTXQueueCount();
  if (q > s_bb_rxh.tx_q_hwm) s_bb_rxh.tx_q_hwm = q;
  __set_PRIMASK(pm);
  return ok;
}

bool can_cone_send(const ODrive::CanFrame& f) {
  if (!partner_recent(&s_cone_last_rx_us, s_cone_rxh)) return false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool ok = send_on(can_cone, f);
  if (ok) s_cone_tx++; else s_cone_rxh.tx_deferred++;
  const uint16_t q = (uint16_t)can_cone.getTXQueueCount();
  if (q > s_cone_rxh.tx_q_hwm) s_cone_rxh.tx_q_hwm = q;
  __set_PRIMASK(pm);
  return ok;
}

bool can_jugglebot_send(const ODrive::CanFrame& f) {
  if (!partner_recent(&s_jugglebot_last_rx_us, s_jugglebot_rxh)) return false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool ok = send_on(can_jugglebot, f);
  if (ok) s_jugglebot_tx++; else s_jugglebot_rxh.tx_deferred++;
  const uint16_t q = (uint16_t)can_jugglebot.getTXQueueCount();
  if (q > s_jugglebot_rxh.tx_q_hwm) s_jugglebot_rxh.tx_q_hwm = q;
  __set_PRIMASK(pm);
  return ok;
}

// Shared CAN3 command gate (declared in can_buses.h; consumed by rpc.cpp leg
// frames + platform_relay reads/writes). WARN/BUS_OFF → refuse; OK/UNKNOWN allow.
//
// Since 2026-07-29 this reads classify_command_gate(), NOT the classify_bus_health()
// value that feeds the uplink: the gate acts only on SUSTAINED error-passive
// confinement, while the reported health stays instantaneous and honest. The full
// root-cause rationale (positive feedback via the TEC decay pump; one predicate
// amplifying a low-rate wire-error source into a 42 %-duty outage across every CAN3
// consumer) is at classify_command_gate in can_buses.h. BUS_OFF and RX staleness are
// unchanged and still act instantly.
bool jugglebot_commands_allowed() {
  const uint8_t g = classify_command_gate(
      atomic_read_u64(&s_jugglebot_last_rx_us),   // interval: *_last_rx_us are mono
      micros64(),
      s_jugglebot_rxh.flt_live,        // single-byte volatile: atomic on Cortex-M7
      s_jugglebot_rxh.flt_sustained);  // same, written only by task_can_rx
  return g != JbUdp::BusHealth::WARN && g != JbUdp::BusHealth::BUS_OFF;
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
  // cone_health reads the ID-DISCRIMINATED cone stamp, NOT the shared gate stamp:
  // with a clapboard attached instead, this must report UNKNOWN ("no catching cone
  // has ever been seen") rather than OK. The wire field's SEMANTICS are unchanged —
  // it still means "catching cone" — which is what lets existing consumers keep
  // reading it. The bus's own liveness is not lost: it moves to clapboard_present.
  // flt_live stays the cone BUS's confinement state; confinement is a property of
  // the controller, not of which peripheral is plugged into it.
  // KNOWN RESIDUAL, accepted and unchanged by this edit: classify_bus_health
  // short-circuits to UNKNOWN when last_rx_us == 0, BEFORE it looks at flt_live —
  // so while a clapboard (or nothing) is attached, a live WARN/BUS_OFF on the cone
  // controller does not reach this row. That is the PRE-EXISTING "device absent ⇒
  // UNKNOWN" semantics, not something introduced here: with no cone plugged in at
  // all, bus3_health has always read UNKNOWN through a bus-off. Fixing it means
  // editing the classifier all three buses share, which is a far wider blast radius
  // than this phase; the cone bus carries no safety-critical traffic, and
  // clapboard_present below still answers "is the bus alive".
  s.cone_health = health_of(atomic_read_u64(&s_cone_only_last_rx_us), s_cone_rxh.flt_live);
  s.jugglebot_health = health_of(atomic_read_u64(&s_jugglebot_last_rx_us), s_jugglebot_rxh.flt_live);
  // Clapboard presence: same staleness term classify_bus_health uses for WARN, so
  // the bit and cone_health cannot disagree about how stale is stale. Deliberately
  // NOT the 5 s BUS_PARTNER_STALENESS_US window — that one exists to keep an
  // un-ACKed TX off a partner-less bus and is far too slow to be an operator-facing
  // "is it plugged in" answer against a 10 Hz clapboard heartbeat.
  {
    const uint64_t clap = atomic_read_u64(&s_clap_last_rx_us);
    s.clapboard_present = (clap != 0 &&
                           (micros64() - clap) <= CAN_HEARTBEAT_TIMEOUT_US) ? 1u : 0u;
  }
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
  o.tx_deferred = h.tx_deferred;
  o.tx_q_hwm    = h.tx_q_hwm;
  // 64-bit: read atomically (torn-load guard). Every other field here is a single
  // word, but a plain two-word load of this one could tear across a task_can_rx
  // preemption — same discipline as the *_last_rx_us reads in can_buses_stats().
  o.flt_passive_since_us = atomic_read_u64(&h.flt_passive_since_us);
  o.flt_sustained        = h.flt_sustained;
  return o;
}

CanRxHealth can_buses_rx_health() {
  CanRxHealth s;
  s.bb        = snapshot_bus(s_bb_rxh);
  s.cone      = snapshot_bus(s_cone_rxh);
  s.jugglebot = snapshot_bus(s_jugglebot_rxh);
  s.decode_short    = s_decode_short;
  s.decode_bad_axis = s_decode_bad_axis;
  // Single-word loads, one per axis. A torn read ACROSS axes is harmless here for
  // the same reason snapshot_bus gives: these are differenced over a 1 s window,
  // and an axis sampled one frame early simply moves that frame into the next
  // window — it cannot manufacture a stall, which is the only conclusion drawn.
  for (uint8_t i = 0; i < NUM_AXES; ++i) s.enc_frames[i] = s_enc_frames[i];
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
