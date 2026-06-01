/*****************************************************************************************
 *  Teensy 4.1 — Jugglebot "leg-bridge" microcontroller (NEW)
 *  ------------------------------------------------------------------
 *  Owns all leg CAN responsibility, offloaded from the Jetson. See
 *  plans/active/teensy-can-offload.md and the firmware-WIP handoff doc.
 *
 *  Stack: FreeRTOS (FreeRTOS_TEENSY4) + QNEthernet (lwIP) + FlexCAN_T4 (x2).
 *
 *  Topology:
 *    Jetson  <--UDP/Ethernet 192.168.42.0/30-->  THIS Teensy
 *    THIS Teensy  <--CAN1 (shared)-->  hand ODrive, platform Teensy 4.0, BB, cone
 *    THIS Teensy  <--CAN2 (private)-->  6 leg ODrives
 *
 *  Roles: time-sync MASTER on CAN1 0x7DD (replaces the Jetson); 500 Hz Hermite
 *  interpolator → leg setpoints on CAN2; telemetry/diagnostics uplink; fault
 *  state machine with the can_node deferred-stow safety inversion preserved.
 *
 *  This file is the FreeRTOS scaffold: it brings up Ethernet, creates the tasks,
 *  and starts the scheduler. Subsystem logic lives in the sibling modules.
 *  PHASE markers below show where each migration phase plugs in.
 *****************************************************************************************/

#include <Arduino.h>
#include "freertos_shim.h"

#include "legbridge_config.h"
#include "udp_protocol.h"
#include "time_base.h"
#include "net_ethernet.h"
#include "udp_link.h"
#include "can_buses.h"           // Phase 5
#include "time_sync_master.h"    // Phase 5
#include "rpc.h"                 // Phase 5
#include "axis_state.h"          // Phase 6 (cache populated by CAN RX)
#include "telemetry.h"           // Phase 6
#include "leg_interp.h"          // Phase 7
#include "fault_machine.h"       // Phase 8
// PROFILING: #include "profiling.h"

using namespace LegBridge;

// ─────────────────────────────────────────────────────────────────────────────
//  Link health (updated by the Jetson heartbeat handler)
// ─────────────────────────────────────────────────────────────────────────────
static volatile uint64_t g_last_jetson_hb_us = 0;

static void on_jetson_heartbeat(uint16_t /*seq*/, const uint8_t* payload, uint16_t len) {
  if (len < JbUdp::HEARTBEAT_J2T_SIZE) return;
  g_last_jetson_hb_us = now_wall_us();
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
  if (g_last_jetson_hb_us == 0) return JbUdp::LinkState::INIT;
  const uint64_t age = now_wall_us() - g_last_jetson_hb_us;
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
  p.bus1_health = cs.can1_health;
  p.bus2_health = cs.can2_health;
  p.fault_state = fault_state();
  p.flags       = (time_synced() ? 0x1u : 0x0u)
                | (fault_stow_pending() ? 0x2u : 0x0u)
                | (all_axis_heartbeats_ok() ? 0x4u : 0x0u);
  p.uptime_ms   = (uint32_t)(micros64() / 1000ULL);
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

// CAN RX/TX servicing (priority 5). Pumps both buses' events() so onReceive
// callbacks decode into the cache and TX mailboxes drain.
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
// diagnostics.
static void task_telem(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / TELEM_RATE_HZ);
  for (;;) {
    telemetry_step();
    vTaskDelayUntil(&last, period);
  }
}

// Fault state machine (priority 3) at FAULT_TASK_HZ. Error eval + CAN2 watchdog
// + deferred stow + guard-mode/output gate.
static void task_fault(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000 / FAULT_TASK_HZ);
  for (;;) {
    fault_step();
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

// Diagnostics / LED heartbeat (priority 1, lowest). Blinks the LED at 1 Hz so a
// bench operator can see the scheduler is alive; prints a one-line status.
static void task_diag(void*) {
  TickType_t last = xTaskGetTickCount();
  bool on = false;
  for (;;) {
    on = !on;
    digitalWriteFast(LED_PIN, on);
    if (on) {
      const UdpStats s = udp_link_stats();
      Serial.printf("[diag] link=%u rx=%lu tx=%lu crc_err=%lu seq_gaps=%lu synced=%d heap=%u\n",
                    link_state(), (unsigned long)s.rx_frames, (unsigned long)s.tx_frames,
                    (unsigned long)s.crc_errors, (unsigned long)s.seq_gaps,
                    (int)time_synced(), (unsigned)xPortGetFreeHeapSize());
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
  can_buses_init();                // Phase 5: CAN1 (shared) + CAN2 (legs)
  telemetry_init();                // Phase 6
  fault_machine_init();            // Phase 8 (before interp so the output gate is off at boot)
  leg_interp_init();               // Phase 7: starts the 500 Hz IntervalTimer ISR

  // Create tasks. (Higher number = higher priority in FreeRTOS.)
  xTaskCreate(task_can_rx,    "canrx", STACK_CAN_RX,    nullptr, PRIO_CAN_RX,    nullptr);
  xTaskCreate(task_time_sync, "tsync", STACK_TIME_SYNC, nullptr, PRIO_TIME_SYNC, nullptr);
  xTaskCreate(task_net,       "net",   STACK_UDP_RX,    nullptr, PRIO_UDP_RX,    nullptr);
  xTaskCreate(task_fault,     "fault", STACK_FAULT,     nullptr, PRIO_FAULT,     nullptr);
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
