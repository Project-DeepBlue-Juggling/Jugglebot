#pragma once
// =============================================================================
//  legbridge_config.h — firmware-wide configuration for the leg-bridge Teensy
// =============================================================================
//  Hand-authored (NOT codegen). Pulls protocol/hardware constants from the
//  generated headers and adds firmware-only configuration: pin map, FreeRTOS
//  task table, and the control/safety constants ported 1:1 from the Jetson
//  Python (motor_guard.py / can_node.py) so behaviour is identical.
//
//  Single source of truth for any value that also lives in Python is the
//  generated config; values mirrored here from Python source files are marked
//  with their origin so a future codegen pass (Phase 10) can hoist them.
// =============================================================================

#include <cstdint>
#include "protocol_config.h"   // CAN protocol: CanBus, ODriveCmd, NodeId, SharedCanId, InputScale...
#include "hardware_config.h"   // Geometry, ODrive limits, operational constants
#include "udp_protocol.h"      // JbUdp:: wire protocol (ports, IPs, framing)

namespace LegBridge {

// ── Identity ────────────────────────────────────────────────────────────────
constexpr char     FW_NAME[]    = "jugglebot-legbridge";
constexpr uint16_t FW_VERSION   = 1;          // bump on behavioural change

// ── Network (static, point-to-point /30) ─────────────────────────────────────
// IP octets and ports come from the generated udp_protocol.h (JbUdp::).
constexpr uint8_t  TEENSY_IP[4] = { JbUdp::TEENSY_IP_0, JbUdp::TEENSY_IP_1,
                                    JbUdp::TEENSY_IP_2, JbUdp::TEENSY_IP_3 };
constexpr uint8_t  JETSON_IP[4] = { JbUdp::TEENSY_IP_0, JbUdp::TEENSY_IP_1,
                                    JbUdp::TEENSY_IP_2, JbUdp::JETSON_IP_3 };
constexpr uint8_t  NETMASK[4]   = { 255, 255, 255, 252 };   // /30
// No gateway, no DHCP, no mDNS — see teensy-can-offload.md "Jetson network setup".

// ── CAN bus wiring (Teensy 4.1) ───────────────────────────────────────────────
//  CAN1 = shared bus  (hand ODrive, platform Teensy 4.0, BB, cone)  pins 22(TX)/23(RX)
//  CAN2 = private leg bus (6 leg ODrives)                           pins  1(TX)/ 0(RX)
//  (CAN3 on pins 31/30 is CAN-FD-capable — reserved for the deferred FD path.)
constexpr uint32_t CAN_BITRATE  = CanBus::BAUD_RATE;   // 1 Mbps, all nodes agree
constexpr uint8_t  CAN1_TX_PIN  = 22;
constexpr uint8_t  CAN1_RX_PIN  = 23;
constexpr uint8_t  CAN2_TX_PIN  = 1;
constexpr uint8_t  CAN2_RX_PIN  = 0;

// ── Status LED ────────────────────────────────────────────────────────────────
constexpr uint8_t  LED_PIN      = 13;          // Teensy on-board LED

// ── Axis layout (mirrors odrive.py groupings) ────────────────────────────────
constexpr uint8_t  NUM_LEGS     = JbUdp::NUM_LEGS;   // 6, node ids 0..5
constexpr uint8_t  HAND_AXIS    = NodeId::JUGGLEBOT_HAND;  // 6
constexpr uint8_t  NUM_AXES     = JbUdp::NUM_AXES;   // 7 (legs + hand) — cache + telemetry breadth

// ── Loop rates ────────────────────────────────────────────────────────────────
constexpr uint32_t INTERP_RATE_HZ      = 500;     // motor_guard DEFAULT_RATE_HZ
constexpr uint32_t INTERP_PERIOD_US    = 1000000u / INTERP_RATE_HZ;   // 2000 us
constexpr uint32_t TIME_SYNC_RATE_HZ   = 100;     // 0x7DD broadcast (can_node ts_period 0.01)
constexpr uint32_t TIME_SYNC_PERIOD_US = 1000000u / TIME_SYNC_RATE_HZ;
constexpr uint32_t TELEM_RATE_HZ       = 100;     // motor-state uplink
constexpr uint32_t TELEM_PERIOD_US     = 1000000u / TELEM_RATE_HZ;
constexpr uint32_t HEARTBEAT_RATE_HZ   = JbUdp::HEARTBEAT_HZ;   // 10
constexpr uint32_t DIAG_HEARTBEAT_HZ   = 1;       // 1 Hz forced diagnostic/profile refresh
constexpr uint32_t FAULT_TASK_HZ       = 10;      // can_node fault cadence
constexpr uint32_t WATCHDOG_HZ         = 1;       // can_node 1 Hz heartbeat watchdog

// Time-of-day drift re-sync interval (teensy-can-offload.md: every 10-60 s).
constexpr uint32_t TIMEOFDAY_RESYNC_MS = 30000u;  // 30 s

// ── FreeRTOS task table (priorities + stacks) ────────────────────────────────
//  Priority 6 (interp) is the HIGHEST — nothing preempts the 500 Hz tick.
//  Stacks are in StackType_t WORDS (4 bytes on Cortex-M7). The plan budgets
//  bytes; *_STACK is the word count = bytes / 4.
constexpr uint8_t PRIO_INTERP      = 6;   // leg_interp (hard deadline)
constexpr uint8_t PRIO_CAN_TX      = 5;   // can2_tx
constexpr uint8_t PRIO_CAN_RX      = 5;   // can1_rx / can2_rx
constexpr uint8_t PRIO_UDP_RX      = 4;   // usb_rx (UDP downlink)
constexpr uint8_t PRIO_TIME_SYNC   = 4;   // time_sync_master
constexpr uint8_t PRIO_UDP_TX      = 3;   // usb_tx (telemetry uplink)
constexpr uint8_t PRIO_FAULT       = 3;   // fault_state
constexpr uint8_t PRIO_WATCHDOG    = 2;   // watchdog
constexpr uint8_t PRIO_DIAG        = 1;   // diag / profiling

constexpr uint16_t STACK_INTERP    = 2048 / 4;   // 512 words
constexpr uint16_t STACK_CAN_TX    = 1024 / 4;
constexpr uint16_t STACK_CAN_RX    = 2048 / 4;
constexpr uint16_t STACK_UDP_RX    = 4096 / 4;
constexpr uint16_t STACK_TIME_SYNC = 1024 / 4;
constexpr uint16_t STACK_UDP_TX    = 4096 / 4;
constexpr uint16_t STACK_FAULT     = 2048 / 4;
constexpr uint16_t STACK_WATCHDOG  = 1024 / 4;
constexpr uint16_t STACK_DIAG      = 1024 / 4;

// ── Interpolator constants (PORTED 1:1 from motor_guard.py) ──────────────────
//  Source: ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py constants block.
constexpr float    SEGMENT_T_S          = 0.025f;  // _mpc_segment_T (nominal MPC fine step)
constexpr float    MAX_EXTRAP_DT_S      = 0.05f;   // MAX_EXTRAP_DT_S
constexpr float    EXTRAP_DECAY_DT_S    = 0.06f;   // EXTRAP_DECAY_DT_S
constexpr float    JERK_EMA_ALPHA       = 0.3f;    // JERK_EMA_ALPHA
constexpr float    MAX_LEAD_REV         = 0.15f;   // MAX_LEAD_REV lead-clamp
constexpr float    MAX_DEVIATION_REV    = 0.5f;    // MAX_DEVIATION_REV (E-stop)
// MAX_MOTOR_VEL_RPS = ODRIVE_TRAP_VEL_LIMIT_RPS * 1.1 (10% noise margin).
constexpr float    MAX_MOTOR_VEL_RPS    = ODriveDefaults::TRAP_VEL_LIMIT_RPS * 1.1f;  // 16.5

// ── Staleness / liveness thresholds (PORTED 1:1) ─────────────────────────────
//  motor_guard.py: MPC_CMD_STALENESS_S=0.25, MOTOR_FB_STALENESS_S=0.15.
//  can_node.py:   _HEARTBEAT_TIMEOUT_S=2.0 (CAN heartbeat watchdog).
constexpr uint32_t MPC_CMD_STALENESS_US = 250000u;   // 0.25 s → E-STOP (link-fault trigger)
constexpr uint32_t MOTOR_FB_STALENESS_US = 150000u;  // 0.15 s → suppress commands
constexpr uint32_t CAN_HEARTBEAT_TIMEOUT_US = 2000000u;  // 2.0 s → fatal_can_error
// Jetson UDP link: declare lost after LINK_LOST_MISSES missed heartbeats.
constexpr uint32_t JETSON_LINK_TIMEOUT_US =
    (1000000u / HEARTBEAT_RATE_HZ) * JbUdp::LINK_LOST_MISSES;   // 500 ms

// ── Fault state machine constants (PORTED 1:1 from can_node.py / motor_state.py) ─
//  motor_state.py: max_soft_reset_attempts = 1 (the cap exists because of a real
//  soft-reset bounce-loop incident — DO NOT raise it without that context).
constexpr uint8_t  MAX_SOFT_RESET_ATTEMPTS = 1;
//  odrive.py ERR_DC_BUS_UNDER_VOLTAGE = 512.
constexpr uint32_t ERR_DC_BUS_UNDER_VOLTAGE = 512u;

// ── Per-leg stroke clamp bounds (rev) ────────────────────────────────────────
//  Backstop matching motor_guard's _stroke_min_rev / _stroke_max_rev, which are
//  WorkspaceLimits.from_geometry(leg_hard_{min,max}_mm) * GEOM_MM_TO_REV (per-leg).
//  Captured 2026-06-01 from the live MotorGuard; the xref harness asserts these
//  equal the running guard's values so drift is caught.
//  TODO(Phase 10): hoist into codegen (derive from workspace limits in YAML).
constexpr float STROKE_MIN_REV[NUM_LEGS] =
    { 0.070917f, 0.070954f, 0.070448f, 0.070934f, 0.071340f, 0.071248f };
constexpr float STROKE_MAX_REV[NUM_LEGS] =
    { 3.900413f, 3.902459f, 3.874629f, 3.901381f, 3.923703f, 3.918615f };

// ── ODrive position clip bounds (odrive.clip_position) ───────────────────────
constexpr float LEG_MOTOR_MAX_POSITION  = Geometry::LEG_MOTOR_MAX_POSITION_REVS;   // 4.2
constexpr float HAND_MOTOR_MAX_POSITION = Geometry::HAND_MOTOR_MAX_POSITION_REVS;  // 11.1

// ── ODrive int16 input scaling (matches protocol_config InputScale) ──────────
constexpr float LEG_VEL_SCALE  = InputScale::leg_vel;   // 1000.0
constexpr float LEG_TOR_SCALE  = InputScale::leg_tor;   // 10000.0
constexpr float HAND_VEL_SCALE = InputScale::hand_vel;  // 100.0
constexpr float HAND_TOR_SCALE = InputScale::hand_tor;  // 100.0

}  // namespace LegBridge
