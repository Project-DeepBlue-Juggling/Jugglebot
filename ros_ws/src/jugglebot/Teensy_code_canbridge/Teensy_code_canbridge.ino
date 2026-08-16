/*****************************************************************************************
 *  Teensy 4.1 — Jugglebot "can-bridge" microcontroller (NEW)
 *  ------------------------------------------------------------------
 *  Owns all leg CAN responsibility, offloaded from the Jetson. See
 *  the can-hub bring-up logbook entry (2026-06-06-can-hub-bringup).
 *
 *  Stack: FreeRTOS (FreeRTOS_TEENSY4) + QNEthernet (lwIP) + FlexCAN_T4 (x3).
 *
 *  Topology (three subsystem-isolated CAN buses, ADR-0013):
 *    Jetson  <--UDP/Ethernet 192.168.42.0/30-->  THIS Teensy
 *    THIS Teensy  <--CAN1 (Ball Butler)-->  BB Teensy
 *    THIS Teensy  <--CAN2 (cone)-->  catching cone Teensy (often disconnected)
 *    THIS Teensy  <--CAN3 (Jugglebot core)-->  6 leg ODrives + Hand ODrive +
 *                                              platform Teensy 4.0
 *
 *  Roles: time-sync MASTER broadcasting 0x7DD on all three buses (replaces the
 *  Jetson); 500 Hz Hermite interpolator → leg setpoints on CAN3; telemetry/
 *  diagnostics uplink; fault state machine with the can_node deferred-stow
 *  safety inversion preserved.
 *
 *  This file is the FreeRTOS scaffold: it brings up Ethernet, creates the tasks,
 *  and starts the scheduler. Subsystem logic lives in the sibling modules.
 *  The include list below notes which subsystem each sibling module owns.
 *****************************************************************************************/

#include <Arduino.h>
#include "freertos_shim.h"

// tsandmann/freertos-teensy scopes Arduino pin constants (OUTPUT/LOW/HIGH/etc.)
// into the `arduino::` namespace via its `arduino_freertos.h`. Hoist them so
// `pinMode(LED_PIN, OUTPUT)` etc. compile unmodified.
using namespace arduino;

#include "canbridge_config.h"
#include "udp_protocol.h"
#include "time_base.h"
#include "net_ethernet.h"
#include "udp_link.h"
#include "can_buses.h"           // three subsystem CAN buses (CAN1/2/3)
#include "time_sync_master.h"    // 0x7DD time-sync master
#include "clap_link.h"           // electronic clapboard: 2 Hz CLAP_LINK beacon (cone bus)
#include "rpc.h"                 // Jetson->Teensy RPC server
#include "axis_state.h"          // per-axis state cache (populated by CAN RX)
#include "ball_butler_state.h"   // BB heartbeat cache (populated by CAN1 RX)
#include "telemetry.h"           // motor-state + diagnostics uplink
#include "leg_interp.h"          // 500 Hz leg setpoint interpolator
#include "fault_machine.h"       // fault state machine + CAN3 watchdog
#include "leg_homing.h"          // leg homing state machine
#include "leg_activate.h"        // leg activate (move to active pose)
#include "leg_deactivate.h"      // leg deactivate (controlled lower + IDLE)
#include "version_check.h"       // Get_Version sweep + version cache
#include "gpio_poll.h"           // hand ball-present sensor poll (hand ODrive G02)
#include "hand_ops.h"            // hand traj conduit — the [handphase] diag ring
#include "profiling.h"           // Profiling/instrumentation

using namespace CanBridge;

// ─────────────────────────────────────────────────────────────────────────────
//  Link health (updated by the Jetson heartbeat handler)
// ─────────────────────────────────────────────────────────────────────────────
static volatile uint64_t g_last_jetson_hb_us = 0;

static void on_jetson_heartbeat(uint16_t /*seq*/, const uint8_t* payload, uint16_t len) {
  if (len < sizeof(JbUdp::HeartbeatJ2TPayload)) return;
  atomic_write_u64(&g_last_jetson_hb_us, micros64());   // 64-bit monotonic; read as an interval by link_state()
  JbUdp::HeartbeatJ2TPayload p;
  memcpy(&p, payload, sizeof(p));
  fault_set_mpc_active((p.flags & 0x1u) != 0);   // bit0 = MPC commanding (guard ENABLED)
}

// True iff every axis has been seen and none is heartbeat-stale.
static bool all_axis_heartbeats_ok() {
  const uint64_t now = micros64();   // interval clock: heartbeat freshness
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    if (!axes[i].heartbeat_seen) return false;
    if (now - atomic_read_u64(&axes[i].last_heartbeat_us) > CAN_HEARTBEAT_TIMEOUT_US) return false;
  }
  return true;
}

static uint8_t link_state() {
  if (!net_link_up()) return JbUdp::LinkState::INIT;
  const uint64_t last = atomic_read_u64(&g_last_jetson_hb_us);
  if (last == 0) return JbUdp::LinkState::INIT;
  const uint64_t age = micros64() - last;   // interval: g_last_jetson_hb_us is mono
  if (age > JETSON_LINK_TIMEOUT_US) return JbUdp::LinkState::LOST;
  return JbUdp::LinkState::UP;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Heartbeat uplink (T→J) — also our liveness beacon
// ─────────────────────────────────────────────────────────────────────────────
static void send_heartbeat_t2j() {
  const CanStats cs = can_buses_stats();
  JbUdp::HeartbeatT2JPayload p{};
  p.t_teensy_us = now_wall_us();   // wire-bound absolute timestamp — wall by contract
  p.link_state  = link_state();
  // Two on-wire health slots, three buses: the safety-critical
  // Jugglebot core bus takes slot 1, Ball Butler slot 2. The wire field NAMES
  // (bus1_health/bus2_health) are fixed by udp_protocol.h and stay unchanged.
  // TODO(cone-health-uplink): expose cone (CAN2) health on the uplink in the next
  // protocol codegen update (a third health slot).
  p.bus1_health = cs.jugglebot_health;   // CAN3 (Jugglebot core)
  p.bus2_health = cs.bb_health;          // CAN1 (Ball Butler)
  p.fault_state = fault_state();
  namespace HF = JbUdp::HeartbeatT2JFlags;   // generated single source for these bits
  p.flags       = (time_synced() ? HF::TIME_SYNCED : 0u)
                | (fault_stow_pending() ? HF::STOW_PENDING_ON_RECONNECT : 0u)
                | (all_axis_heartbeats_ok() ? HF::ALL_AXIS_HEARTBEATS_OK : 0u)
                | (fault_mpc_active() ? HF::MPC_ACTIVE : 0u)   // bit3:
                                                        // firmware-side mpc_active — a setpoint source
                                                        // can verify its arm actually took (catches
                                                        // a competing heartbeat authority).
                | (((uint32_t)interp_torque_clamp_mask()
                    << JbUdp::HEARTBEAT_TORQUE_CLAMP_SHIFT)
                   & HF::TORQUE_CLAMP_MASK);            // bits 8-13: per-leg torque_ff
                                                        // ingest-clamp mask from the last ACCEPTED
                                                        // setpoint (mirrors lead_clamp_mask below;
                                                        // 2026-07-14 gravity-FF observability).
  // bits 4-5: cone (CAN2) BusHealth — closes the cone-health-uplink TODO above.
  // Since 2026-08-16 this is the CATCHING CONE's health specifically: on_cone_rx
  // discriminates the arbitration id, so an attached electronic clapboard no
  // longer reports a cone that is not there.
  p.flags      |= ((uint32_t)cs.cone_health << JbUdp::HEARTBEAT_CONE_HEALTH_SHIFT)
                  & HF::CONE_HEALTH_MASK;
  // bit 6: an electronic clapboard is on that same bus instead. The two devices
  // are mutually exclusive by physical connection, so this is the other half of
  // the answer — without it, "cone UNKNOWN" cannot be told from "bus empty".
  p.flags      |= (cs.clapboard_present ? HF::CLAPBOARD_PRESENT : 0u);
  p.uptime_ms   = (uint32_t)(micros64() / 1000ULL);

  // Ball Butler heartbeat snapshot (replaces legacy can_node bb/
  // heartbeat publisher). snapshot_bb() takes a seqlock-consistent view so the
  // multi-field copy never reads a torn write from the CAN1 RX decode.
  BallButlerSnapshot bb{};
  snapshot_bb(bb_state, bb);
  p.bb_state      = bb.state;
  p.bb_state_data = bb.state_data;
  p.bb_flags      = (uint8_t)((bb.ball_in_hand    ? 0x1u : 0x0u)
                            | (bb.heartbeat_seen  ? 0x2u : 0x0u)
                            | (bb.heartbeat_stale ? 0x4u : 0x0u));
  p.bb_yaw_deg    = bb.yaw_deg;
  p.bb_pitch_deg  = bb.pitch_deg;
  p.bb_hand_mm    = bb.hand_mm;

  // Leg guard-deviation diagnostics (2026-07-10 forensics). Live per-leg
  // deviation (u0 - encoder — the exact MAX_DEVIATION guard quantity) + the
  // lead-clamp bitmask from the last 500 Hz interp tick, plus the frozen
  // latch-event snapshot. Individual fields are single-word reads (float/uint8,
  // atomic on the M7); a rare torn multi-field snapshot is acceptable for 10 Hz
  // diagnostics and cannot affect any safety decision (these are report-only).
  for (uint8_t i = 0; i < JbUdp::NUM_LEGS; ++i)
    p.live_deviation[i] = interp_base_pos(i) - axes[i].pos_rev;
  p.lead_clamp_mask = interp_lead_clamp_mask();
  p.max_dev_leg     = fault_max_dev_leg();
  p.max_dev_value   = fault_max_dev_value();
  p.max_dev_u0      = fault_max_dev_u0();
  p.max_dev_enc     = fault_max_dev_enc();

  udp_send_stream(JbUdp::MsgType::HEARTBEAT_T2J, (const uint8_t*)&p, sizeof(p));
}

// ─────────────────────────────────────────────────────────────────────────────
//  TASKS
// ─────────────────────────────────────────────────────────────────────────────

// UDP RX / network service (priority 4). Pumps lwIP + dispatches downlink.
static void task_net(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    udp_link_service();
    // 1 ms cadence: well above the 40 Hz downlink + 10 Hz heartbeat rates while
    // leaving the CPU to the higher-priority CAN/interp tasks.
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
  }
}

// CAN RX/TX servicing (priority 5). Pumps all three buses' events() so onReceive
// callbacks decode into the cache (CAN3) / count (CAN1, CAN2) and TX mailboxes drain.
static void task_can_rx(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    can_buses_service();
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1));   // 1 kHz CAN service (matches can_node poll)
  }
}

// Time-sync master (priority 4) at TIME_SYNC_RATE_HZ. Broadcasts 0x7DD, paces
// the time-of-day query, and carries the clapboard's 2 Hz CLAP_LINK beacon.
static void task_time_sync(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / TIME_SYNC_RATE_HZ);
  for (;;) {
    time_sync_step();
    // Rides this task because it is the cone bus's other 100 Hz producer, so the
    // beacon and the 0x7DD broadcast can never interleave with each other from
    // two contexts. link_state() is passed IN rather than read inside clap_link:
    // the predicate stays single-sourced and the new TU stays natively testable
    // (clap_link.h states the full argument).
    clap_link_step(link_state());
    vTaskDelayUntil(&last, period);
  }
}

// Telemetry uplink (priority 3) at TELEM_RATE_HZ. 100 Hz motor state + on-change
// diagnostics + the cone CAN2→CONE_FRAME relay (cone uplink) + the BB
// CAN1 CMD_RESULT→host relay (loud command-outcome channel).
static void task_telem(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / TELEM_RATE_HZ);
  for (;;) {
    telemetry_step();
    cone_uplink_step();
    cmd_result_uplink_step();
    platform_uplink_step();   // Platform-Teensy relay reply uplink
    hand_cmd_echo_uplink_step(); // hand Set_Input_Pos command-echo
    hand_sensor_uplink_step();   // hand ball-sensor state (per new reply + 1 Hz keepalive)
    can_errors_uplink_step();    // CAN3 wire-error + confinement counters @ 1 Hz
    bridge_tx_diag_uplink_step();  // per-bus TX deferral/queue + hand-stage exits @ 1 Hz
    cache_diag_uplink_step();      // encoder-cache age floor/peak + CAN RX ring @ 1 Hz
                                   // (samples the ages EVERY tick — see telemetry.h)
    ring_diag_uplink_step();       // TRUE RX-ring occupancy vs reported `_available`
                                   // (= the FlexCAN_T4 leak) + jb delivery lag + SDO RTT @ 1 Hz
    bridge_identity_uplink_step(); // FW_VERSION + PROTOCOL_VERSION echo @ 1 Hz
    vTaskDelayUntil(&last, period);
  }
}

// Fault state machine (priority 3) at FAULT_TASK_HZ. Error eval + CAN3 watchdog
// + deferred stow + guard-mode/output gate.
static void task_fault(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / FAULT_TASK_HZ);
  for (;;) {
    fault_step();
    vTaskDelayUntil(&last, period);
  }
}

// Cold-start leg-motion monitor + slow CAN3 pollers (priority 2) at
// HOMING_RATE_HZ. Runs the velocity-limited move-to-hardstop (HOME) and the
// TRAP_TRAJ move to the active pose (ACTIVATE) state machines when an RPC has
// latched a start; both are cheap no-ops otherwise (rare bench/cold-start ops).
// HOME and ACTIVATE each reject a start while the OTHER is active (symmetric),
// and each stays "active" for its whole physical move (homing polls the Iq spike;
// activate's MONITOR waits for the legs to reach + settle), so at most one drives
// at a time for the full duration — not just the fire instant.
static void task_homing(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / HOMING_RATE_HZ);
  for (;;) {
    homing_step();
    activate_step();
    deactivate_step();
    version_check_step();   // bus-paced Get_Version sweep (no-op once swept)
    gpio_poll_step();       // hand ball-sensor SDO poll (rate-limited, ≤1 TX/tick)
    vTaskDelayUntil(&last, period);
  }
}

// Heartbeat uplink (priority 3) at HEARTBEAT_RATE_HZ.
static void task_heartbeat(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / HEARTBEAT_RATE_HZ);
  for (;;) {
    send_heartbeat_t2j();
    vTaskDelayUntil(&last, period);
  }
}

// One RX-health line per bus on the USB Serial console (bench/debug telemetry).
// sync is the LIVE ESR1.SYNCH bit (1 = controller locked onto the bus right now — the
// "is this bus electrically alive" signal); the rest are cumulative/sticky counters —
// see can_buses.h. In a healthy ALIVE bus every field reads sync=1, hwm≈single-digits,
// capHit=0, err=0, rec=0, tec=0, flt=active; anything else points at the failure class
// (sync=0 = no bus/transceiver, task starvation, drain-budget bound, wire errors by type,
// RX/TX error counters climbing toward error-passive, bus-off, or garbled frames). For a
// DISCONNECTED bus partner, watch tec (un-ACKed TX) rising ahead of flt=BUSOFF.
static void print_bus_health(const char* name, const BusRxHealth& b) {
  static const char* const FLT[3] = { "active", "passive", "BUSOFF" };
  // err= counts WIRE-ERROR snapshots only (0 on a healthy bus); chg= is the raw
  // ESR1-change snapshot counter (benign IDLE/RX/TX phase flips included — climbs
  // continuously with traffic; its only diagnostic use is proving the capture
  // machinery is alive). Pre-2026-07-05 captures printed the raw counter AS err=.
  // flt= is the sticky worst-ever confinement; fltNow= is the LIVE FLTCONF that
  // (with RX staleness) drives the health classification the gates + uplink see.
  // defer= / txq= are the TX-path pair (2026-08-02): defer= counts sends whose
  // write() returned -1, i.e. the frame went into the 64-slot software txBuffer
  // instead of a mailbox — DEFERRED, not dropped — and txq= is that buffer's
  // high-water mark, sampled at send instants. txq= at/near 64 is the one that
  // means real loss (overflow overwrites the oldest entry). Existing token names
  // are unchanged ON PURPOSE: the 2026-07-29 A/B discriminator table greps them.
  Serial.printf("[canhealth] %-9s sync=%u hwm=%u capHit=%lu err=%lu flags=0x%02x rec=%u tec=%u flt=%s fltNow=%s gated=%lu defer=%lu txq=%u chg=%lu\n",
                name, (unsigned)b.synced, (unsigned)b.depth_hwm, (unsigned long)b.cap_hits,
                (unsigned long)b.wire_errs, (unsigned)b.err_flags, (unsigned)b.rec_max,
                (unsigned)b.tec_max, FLT[(b.fault_conf < 3) ? b.fault_conf : 2],
                FLT[(b.flt_live < 3) ? b.flt_live : 2],
                (unsigned long)b.tx_gated, (unsigned long)b.tx_deferred,
                (unsigned)b.tx_q_hwm, (unsigned long)b.err_events);
  // Marginal-CAN3 diagnostic (2026-07-05): per-type error-snapshot counters, TX/RX
  // capture context, and the live ECR counters + their positive-delta sums. Printed
  // only for a bus that has seen a wire error or TEC/REC movement (healthy buses
  // stay quiet; tec/rec can move without a captured snapshot during a storm, hence
  // the delta-sum terms in the condition).
  if ((b.wire_errs | b.tec_inc_sum | b.rec_inc_sum) != 0) {
    Serial.printf("[canerrs]  %-9s ack=%lu crc=%lu form=%lu stuff=%lu bit0=%lu bit1=%lu"
                  " txctx=%lu rxctx=%lu tecNow=%u recNow=%u tecInc=%lu recInc=%lu\n",
                  name, (unsigned long)b.ack_cnt, (unsigned long)b.crc_cnt,
                  (unsigned long)b.form_cnt, (unsigned long)b.stuff_cnt,
                  (unsigned long)b.bit0_cnt, (unsigned long)b.bit1_cnt,
                  (unsigned long)b.err_tx_ctx, (unsigned long)b.err_rx_ctx,
                  (unsigned)b.tec_live, (unsigned)b.rec_live,
                  (unsigned long)b.tec_inc_sum, (unsigned long)b.rec_inc_sum);
  }
}

// USB Serial console — line-oriented bench commands, the only INPUT the console
// has ever had (it was print-only until the hand ball sensor needed a live
// on/off). Polled from task_diag, so a typed command lands within one diag tick;
// the static line buffer is what lets a line arrive in fragments across ticks.
// Commands are dispatched to their owning module, which prints its own reply.
static void console_step() {
  static char line[32];
  static uint8_t len = 0;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        line[len] = '\0';
        if (!gpio_poll_console(line))
          Serial.printf("[console] unknown command: %s\n", line);
      }
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
    // else: line overflow — the excess is dropped, the prefix still dispatches
  }
}

// Diagnostics / LED heartbeat (priority 1, lowest). Blinks the LED at 1 Hz so a
// bench operator can see the scheduler is alive; prints a one-line status; and
// services the serial console.
static void task_diag(void*) {
  TickType_t last = xTaskGetTickCount();
  bool on = false;
  for (;;) {
    console_step();
    on = !on;
    digitalWriteFast(LED_PIN, on);
    if (on) {
      profiling_step();          // emit the 1 Hz PROFILE frame on the LED-on edge
      // Marginal-CAN3 diagnostic (2026-07-05): decoded bit-timing register dump on
      // the first diag tick (Serial is up by then) and every 60 s after, so a
      // monitor attached mid-run still catches it within a minute.
      {
        static uint32_t timing_tick = 0;
        if (timing_tick % 60u == 0u) can_buses_dump_timing();
        timing_tick++;
      }
      const UdpStats s = udp_link_stats();
      Serial.printf("[diag] link=%u fault=%u rx=%lu tx=%lu crc_err=%lu seq_gaps=%lu drain_cap=%lu synced=%d heap=%u\n",
                    link_state(), fault_state(), (unsigned long)s.rx_frames,
                    (unsigned long)s.tx_frames, (unsigned long)s.crc_errors,
                    (unsigned long)s.seq_gaps, (unsigned long)s.drain_cap_hits,
                    (int)time_synced(), (unsigned)xPortGetFreeHeapSize());
      const CanRxHealth ch = can_buses_rx_health();
      print_bus_health("jugglebot", ch.jugglebot);
      print_bus_health("bb", ch.bb);
      print_bus_health("cone", ch.cone);
      Serial.printf("[canhealth] decode_drops jugglebot: short=%lu bad_axis=%lu  cone_fwd_drops=%lu  cmd_result_fwd_drops=%lu\n",
                    (unsigned long)ch.decode_short, (unsigned long)ch.decode_bad_axis,
                    (unsigned long)can_cone_fwd_drops(),
                    (unsigned long)can_cmd_result_fwd_drops());
      can_buses_print_esr1();   // raw ESR1 words of fresh error snapshots (diagnostic)

      // Hand-dispatch interp-phase stamp (2026-08-09) — the falsifiable test for the
      // phase-locked-dispatch-quantisation verdict (logbook 2026-08-02 addendum § A3,
      // contract in hand_ops.h). Each sample is the dispatch's µs offset within the
      // 2 ms interp cycle plus the stage it exited at. PRE-REGISTERED READ: two tight
      // clusters ⇒ model confirmed; a uniform spread over 0-2000 ⇒ model REFUTED and
      // the mailbox-occupancy story re-opens. Printed ON-CHANGE like print_esr1_ring,
      // so a healthy idle bench stays quiet and `+N` reports every dispatch since the
      // last line even when N exceeds the 8-deep ring (an overrun is visible, not
      // silent). Console only — nothing here is on the wire.
      {
        static uint32_t handphase_n = 0;
        const HandPhaseRing r = hand_phase_ring();
        if (r.n != handphase_n) {
          const uint32_t fresh = r.n - handphase_n;
          const uint32_t show = (fresh > HAND_PHASE_RING_LEN) ? HAND_PHASE_RING_LEN : fresh;
          Serial.printf("[handphase] +%lu:", (unsigned long)fresh);
          for (uint32_t i = 0; i < show; ++i) {
            const HandPhaseSample& s = r.v[(r.n - show + i) % HAND_PHASE_RING_LEN];
            Serial.printf(" %u/%s", (unsigned)s.phase_us,
                          hand_phase_outcome_name(s.outcome));
          }
          Serial.println();
          handphase_n = r.n;
        }
      }

      // Per-axis "are all ODrives responding?" line (USB Serial bench/debug, alongside
      // the [canhealth] lines — NOT on the UDP uplink yet). Columns: legs 0..5 then
      // H(and); each is state/age-ms + a status mark ('?'=never seen, '!'=heartbeat
      // stale, '*'=active error/disarm, ' '=ok). The fresh=N/7 headline is the one-glance
      // "all responding" check. ODrive state codes: IDLE=1, CLOSED_LOOP=8.
      {
        const uint64_t now = micros64();   // interval clock: diag heartbeat ages
        uint8_t fresh = 0;
        for (uint8_t i = 0; i < NUM_AXES; ++i)
          if (axes[i].heartbeat_seen && !axes[i].heartbeat_stale
              && axes[i].active_errors == 0 && axes[i].disarm_reason == 0) fresh++;
        Serial.printf("[axes] fresh=%u/%u", (unsigned)fresh, (unsigned)NUM_AXES);
        for (uint8_t i = 0; i < NUM_AXES; ++i) {
          const char tag = (i < NUM_LEGS) ? (char)('0' + i) : 'H';
          char mark = ' ';
          uint32_t age_ms = 9999u;
          if (!axes[i].heartbeat_seen) {
            mark = '?';
          } else {
            const uint64_t age = now - atomic_read_u64(&axes[i].last_heartbeat_us);
            age_ms = (age > 9999000ULL) ? 9999u : (uint32_t)(age / 1000ULL);
            if (axes[i].heartbeat_stale) mark = '!';
            else if (axes[i].active_errors != 0 || axes[i].disarm_reason != 0) mark = '*';
          }
          Serial.printf(" %c:s%u/%lu%c", tag, (unsigned)axes[i].axis_state,
                        (unsigned long)age_ms, mark);
        }
        Serial.println();
      }

      // Leg output-gate diagnostic (bench): is the firmware armed and
      // actually streaming setpoints to CAN3? mpc_active = J→T heartbeat bit0 seen;
      // guard_mode 0=DISABLED/1=ENABLED/2=ESTOP; output = interp gate (1 ⇒ sending);
      // sp_age_ms = age of the last setpoint from the Jetson (huge ⇒ not receiving);
      // u0 = the latched commanded base position for axis 0.
      Serial.printf("[guard] mpc_active=%u guard_mode=%u output=%u sp_age_ms=%lu u0=%.4f\n",
                    (unsigned)fault_mpc_active(), (unsigned)fault_guard_mode(),
                    (unsigned)interp_output_enabled(),
                    (unsigned long)((micros64() - interp_last_setpoint_us()) / 1000ULL),   // interval: setpoint stamped mono
                    (double)interp_base_pos(0));

      // Per-Ball-Butler state line. Format mirrors [axes]:
      //   [bb] state=<name> ball=<0|1> yaw=<deg> pitch=<deg> hand=<mm> age=<ms><mark>
      // Marks: '?' = never seen, '!' = stale (> BB_HEARTBEAT_TIMEOUT_US since last RX),
      // '*' = BB ERROR state, ' ' = ok. The snapshot pattern matches the [axes] read —
      // we take a seqlock-consistent snapshot so a writer mid-update never produces a
      // torn print.
      {
        BallButlerSnapshot bb{};
        snapshot_bb(bb_state, bb);
        const uint64_t now = micros64();   // interval clock: diag heartbeat ages
        const char* state_names[] = {
            "BOOT", "IDLE", "TRACKING", "THROWING",
            "RELOADING", "CAL", "CHKBALL"};  // states 0..6
        const char* sname =
            (bb.state == BallButlerState::ERROR) ? "ERROR"
            : (bb.state < 7) ? state_names[bb.state] : "?";
        char mark = ' ';
        uint32_t age_ms = 9999u;
        if (!bb.heartbeat_seen) {
          mark = '?';
        } else {
          const uint64_t age = now - bb.last_heartbeat_us;
          age_ms = (age > 9999000ULL) ? 9999u : (uint32_t)(age / 1000ULL);
          if (bb.heartbeat_stale) mark = '!';
          else if (bb.state == BallButlerState::ERROR) mark = '*';
        }
        Serial.printf("[bb] state=%s ball=%u yaw=%.1f pitch=%.1f hand=%.1f age=%lums%c\n",
                      sname, (unsigned)bb.ball_in_hand,
                      (double)bb.yaw_deg, (double)bb.pitch_deg, (double)bb.hand_mm,
                      (unsigned long)age_ms, mark);
      }

      // Hand ball-sensor poller: silent unless the Get_Version gate has parked it
      // on a firmware mismatch, in which case one loud line per tick. It prints
      // HERE and not on task_homing because STACK_DIAG is the only stack sized for
      // Serial formatting (canbridge_config.h:135).
      gpio_poll_diag_step();
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(500));   // toggle every 500 ms → 1 Hz blink
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWriteFast(LED_PIN, LOW);

  // Bring up Ethernet at the static IP (boots the PHY out of reset — only now
  // do the Jetson-side adapter link lights start blinking).
  const bool link = net_ethernet_begin();
  Serial.printf("[boot] %s v%u  eth link=%d  ip=%u.%u.%u.%u\n",
                FW_NAME, FW_VERSION, (int)link,
                TEENSY_IP[0], TEENSY_IP[1], TEENSY_IP[2], TEENSY_IP[3]);

  // Register downlink handlers before the scheduler starts.
  udp_on_heartbeat_j2t(on_jetson_heartbeat);
  Rpc::rpc_server_init();          // Jetson→Teensy ODrive RPCs
  time_sync_master_init();         // time-of-day RPC client
  udp_on_setpoint(interp_on_setpoint);   // 40 Hz setpoint downlink

  udp_link_init();
  can_buses_init();                // CAN1 bb + CAN2 cone + CAN3 jugglebot
  telemetry_init();                // motor-state + diagnostics uplink
  fault_machine_init();            // before interp so the output gate is off at boot
  leg_interp_init();               // starts the 500 Hz IntervalTimer ISR
  homing_init();                   // idle until a HOME RPC latches a start
  activate_init();                 // idle until an ACTIVATE RPC latches a start
  deactivate_init();               // idle until a DEACTIVATE RPC latches a start
  version_check_init();            // clears the version sweep masks
  gpio_poll_init();                // hand ball-sensor poller (boots ON)
  profiling_init();                // instrumentation baselines

  // Create tasks. (Higher number = higher priority in FreeRTOS.)
  xTaskCreate(task_can_rx,    "canrx", STACK_CAN_RX,    nullptr, PRIO_CAN_RX,    nullptr);
  xTaskCreate(task_time_sync, "tsync", STACK_TIME_SYNC, nullptr, PRIO_TIME_SYNC, nullptr);
  xTaskCreate(task_net,       "net",   STACK_UDP_RX,    nullptr, PRIO_UDP_RX,    nullptr);
  xTaskCreate(task_fault,     "fault", STACK_FAULT,     nullptr, PRIO_FAULT,     nullptr);
  xTaskCreate(task_homing,    "home",  STACK_HOMING,    nullptr, PRIO_HOMING,    nullptr);
  xTaskCreate(task_telem,     "telem", STACK_UDP_TX,    nullptr, PRIO_UDP_TX,    nullptr);
  xTaskCreate(task_heartbeat, "hb",    STACK_UDP_TX,    nullptr, PRIO_UDP_TX,    nullptr);
  xTaskCreate(task_diag,      "diag",  STACK_DIAG,      nullptr, PRIO_DIAG,      nullptr);

  Serial.println("[boot] starting FreeRTOS scheduler");
  vTaskStartScheduler();

  // Never reached unless the scheduler fails to start (out of heap, etc.).
  Serial.println("[fatal] scheduler returned — insufficient heap?");
  for (;;) { digitalWriteFast(LED_PIN, HIGH); delay(100); digitalWriteFast(LED_PIN, LOW); delay(100); }
}

void loop() {
  // Unused — FreeRTOS owns execution after vTaskStartScheduler().
}
