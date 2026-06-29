/*****************************************************************************************
 *  Teensy 4.1 — Jugglebot "can-bridge" microcontroller (NEW)
 *  ------------------------------------------------------------------
 *  Owns all leg CAN responsibility, offloaded from the Jetson. See
 *  plans/active/teensy-can-offload.md and the firmware-WIP handoff doc.
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
 *  PHASE markers below show where each migration phase plugs in.
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
#include "can_buses.h"           // Phase 5
#include "time_sync_master.h"    // Phase 5
#include "rpc.h"                 // Phase 5
#include "axis_state.h"          // Phase 6 (cache populated by CAN RX)
#include "ball_butler_state.h"   // Phase A (BB heartbeat cache, populated by CAN1 RX)
#include "telemetry.h"           // Phase 6
#include "leg_interp.h"          // Phase 7
#include "fault_machine.h"       // Phase 8
#include "leg_homing.h"          // Phase 9b
#include "leg_activate.h"        // Phase 11 U5
#include "leg_deactivate.h"      // Phase 11 U5
#include "profiling.h"           // Profiling/instrumentation

using namespace CanBridge;

// ─────────────────────────────────────────────────────────────────────────────
//  Link health (updated by the Jetson heartbeat handler)
// ─────────────────────────────────────────────────────────────────────────────
static volatile uint64_t g_last_jetson_hb_us = 0;

static void on_jetson_heartbeat(uint16_t /*seq*/, const uint8_t* payload, uint16_t len) {
  if (len < sizeof(JbUdp::HeartbeatJ2TPayload)) return;
  atomic_write_u64(&g_last_jetson_hb_us, now_wall_us());   // 64-bit; read by link_state()
  JbUdp::HeartbeatJ2TPayload p;
  memcpy(&p, payload, sizeof(p));
  fault_set_mpc_active((p.flags & 0x1u) != 0);   // bit0 = MPC commanding (guard ENABLED)
}

// True iff every axis has been seen and none is heartbeat-stale.
static bool all_axis_heartbeats_ok() {
  const uint64_t now = now_wall_us();
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    if (!axes[i].heartbeat_seen) return false;
    if (now - axes[i].last_heartbeat_us > CAN_HEARTBEAT_TIMEOUT_US) return false;
  }
  return true;
}

static uint8_t link_state() {
  if (!net_link_up()) return JbUdp::LinkState::INIT;
  const uint64_t last = atomic_read_u64(&g_last_jetson_hb_us);
  if (last == 0) return JbUdp::LinkState::INIT;
  const uint64_t age = now_wall_us() - last;
  if (age > JETSON_LINK_TIMEOUT_US) return JbUdp::LinkState::LOST;
  return JbUdp::LinkState::UP;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Heartbeat uplink (T→J) — also our liveness beacon
// ─────────────────────────────────────────────────────────────────────────────
static void send_heartbeat_t2j() {
  const CanStats cs = can_buses_stats();
  JbUdp::HeartbeatT2JPayload p{};
  p.t_teensy_us = now_wall_us();
  p.link_state  = link_state();
  // Two on-wire health slots, three buses (HANDOFF D4): the safety-critical
  // Jugglebot core bus takes slot 1, Ball Butler slot 2. The wire field NAMES
  // (bus1_health/bus2_health) are fixed by udp_protocol.h and stay unchanged.
  // TODO(phase-10b): expose cone (CAN2) health on the uplink in the next
  // protocol codegen update (a third health slot).
  p.bus1_health = cs.jugglebot_health;   // CAN3 (Jugglebot core)
  p.bus2_health = cs.bb_health;          // CAN1 (Ball Butler)
  p.fault_state = fault_state();
  namespace HF = JbUdp::HeartbeatT2JFlags;   // generated single source for these bits
  p.flags       = (time_synced() ? HF::TIME_SYNCED : 0u)
                | (fault_stow_pending() ? HF::STOW_PENDING_ON_RECONNECT : 0u)
                | (all_axis_heartbeats_ok() ? HF::ALL_AXIS_HEARTBEATS_OK : 0u)
                | (fault_mpc_active() ? HF::MPC_ACTIVE : 0u);  // bit3 (Phase 11):
                                                        // firmware-side mpc_active — a setpoint source
                                                        // can verify its arm actually took (catches
                                                        // a competing heartbeat authority).
  p.uptime_ms   = (uint32_t)(micros64() / 1000ULL);

  // Ball Butler heartbeat snapshot (Phase A — replaces legacy can_node bb/
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

// Time-sync master (priority 4) at TIME_SYNC_RATE_HZ. Broadcasts 0x7DD and paces
// the time-of-day query.
static void task_time_sync(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / TIME_SYNC_RATE_HZ);
  for (;;) {
    time_sync_step();
    vTaskDelayUntil(&last, period);
  }
}

// Telemetry uplink (priority 3) at TELEM_RATE_HZ. 100 Hz motor state + on-change
// diagnostics + the cone CAN2→CONE_FRAME relay (phase-10b cone uplink) + the BB
// CAN1 CMD_RESULT→host relay (Phase-2 loud command-outcome channel).
static void task_telem(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / TELEM_RATE_HZ);
  for (;;) {
    telemetry_step();
    cone_uplink_step();
    cmd_result_uplink_step();
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

// Cold-start leg-motion monitor (priority 2) at HOMING_RATE_HZ. Runs the
// velocity-limited move-to-hardstop (Phase 9b HOME) and the TRAP_TRAJ move to the
// active pose (Phase 11 U5 ACTIVATE) state machines when an RPC has latched a
// start; both are cheap no-ops otherwise (idle the rest of the time — rare bench/
// cold-start ops). HOME and ACTIVATE each reject a start while the OTHER is active
// (symmetric), and each stays "active" for its whole physical move (homing polls
// the Iq spike; activate's MONITOR waits for the legs to reach + settle), so at
// most one drives at a time for the full duration — not just the fire instant.
static void task_homing(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / HOMING_RATE_HZ);
  for (;;) {
    homing_step();
    activate_step();
    deactivate_step();
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
  Serial.printf("[canhealth] %-9s sync=%u hwm=%u capHit=%lu err=%lu flags=0x%02x rec=%u tec=%u flt=%s\n",
                name, (unsigned)b.synced, (unsigned)b.depth_hwm, (unsigned long)b.cap_hits,
                (unsigned long)b.err_events, (unsigned)b.err_flags, (unsigned)b.rec_max,
                (unsigned)b.tec_max, FLT[(b.fault_conf < 3) ? b.fault_conf : 2]);
}

// Diagnostics / LED heartbeat (priority 1, lowest). Blinks the LED at 1 Hz so a
// bench operator can see the scheduler is alive; prints a one-line status.
static void task_diag(void*) {
  TickType_t last = xTaskGetTickCount();
  bool on = false;
  for (;;) {
    on = !on;
    digitalWriteFast(LED_PIN, on);
    if (on) {
      profiling_step();          // emit the 1 Hz PROFILE frame on the LED-on edge
      const UdpStats s = udp_link_stats();
      Serial.printf("[diag] link=%u fault=%u rx=%lu tx=%lu crc_err=%lu seq_gaps=%lu synced=%d heap=%u\n",
                    link_state(), fault_state(), (unsigned long)s.rx_frames,
                    (unsigned long)s.tx_frames, (unsigned long)s.crc_errors,
                    (unsigned long)s.seq_gaps, (int)time_synced(),
                    (unsigned)xPortGetFreeHeapSize());
      const CanRxHealth ch = can_buses_rx_health();
      print_bus_health("jugglebot", ch.jugglebot);
      print_bus_health("bb", ch.bb);
      print_bus_health("cone", ch.cone);
      Serial.printf("[canhealth] decode_drops jugglebot: short=%lu bad_axis=%lu  cone_fwd_drops=%lu  cmd_result_fwd_drops=%lu\n",
                    (unsigned long)ch.decode_short, (unsigned long)ch.decode_bad_axis,
                    (unsigned long)can_cone_fwd_drops(),
                    (unsigned long)can_cmd_result_fwd_drops());

      // Per-axis "are all ODrives responding?" line (USB Serial bench/debug, alongside
      // the [canhealth] lines — NOT on the UDP uplink yet). Columns: legs 0..5 then
      // H(and); each is state/age-ms + a status mark ('?'=never seen, '!'=heartbeat
      // stale, '*'=active error/disarm, ' '=ok). The fresh=N/7 headline is the one-glance
      // "all responding" check. ODrive state codes: IDLE=1, CLOSED_LOOP=8.
      {
        const uint64_t now = now_wall_us();
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
            const uint64_t age = now - axes[i].last_heartbeat_us;
            age_ms = (age > 9999000ULL) ? 9999u : (uint32_t)(age / 1000ULL);
            if (axes[i].heartbeat_stale) mark = '!';
            else if (axes[i].active_errors != 0 || axes[i].disarm_reason != 0) mark = '*';
          }
          Serial.printf(" %c:s%u/%lu%c", tag, (unsigned)axes[i].axis_state,
                        (unsigned long)age_ms, mark);
        }
        Serial.println();
      }

      // Leg output-gate diagnostic (Phase 11 bench): is the firmware armed and
      // actually streaming setpoints to CAN3? mpc_active = J→T heartbeat bit0 seen;
      // guard_mode 0=DISABLED/1=ENABLED/2=ESTOP; output = interp gate (1 ⇒ sending);
      // sp_age_ms = age of the last setpoint from the Jetson (huge ⇒ not receiving);
      // u0 = the latched commanded base position for axis 0.
      Serial.printf("[guard] mpc_active=%u guard_mode=%u output=%u sp_age_ms=%lu u0=%.4f\n",
                    (unsigned)fault_mpc_active(), (unsigned)fault_guard_mode(),
                    (unsigned)interp_output_enabled(),
                    (unsigned long)((now_wall_us() - interp_last_setpoint_us()) / 1000ULL),
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
        const uint64_t now = now_wall_us();
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
  Rpc::rpc_server_init();          // Phase 5: Jetson→Teensy ODrive RPCs
  time_sync_master_init();         // Phase 5: time-of-day RPC client
  udp_on_setpoint(interp_on_setpoint);   // Phase 7: 40 Hz setpoint downlink

  udp_link_init();
  can_buses_init();                // Phase 5: CAN1 bb + CAN2 cone + CAN3 jugglebot
  telemetry_init();                // Phase 6
  fault_machine_init();            // Phase 8 (before interp so the output gate is off at boot)
  leg_interp_init();               // Phase 7: starts the 500 Hz IntervalTimer ISR
  homing_init();                   // Phase 9b (idle until a HOME RPC latches a start)
  activate_init();                 // Phase 11 U5 (idle until an ACTIVATE RPC latches a start)
  deactivate_init();               // Phase 11 U5 (idle until a DEACTIVATE RPC latches a start)
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
